import serial
import time
import os
import sys
import asyncio
import struct

# CONSTANTES
PACKET_LENGTH_BYTES                     =  1
PACKET_DATA_BYTES                       =  16
PACKET_CRC_BYTES                        =  1
PACKET_CRC_INDEX                        =  PACKET_LENGTH_BYTES + PACKET_DATA_BYTES
PACKET_LENGTH                           =  PACKET_LENGTH_BYTES + PACKET_DATA_BYTES + PACKET_CRC_BYTES

PACKET_ACK_DATA0                        =  0x15 
PACKET_RETX_DATA0                       =  0x19

BL_PACKET_SYNC_OBSERVED_DATA0           =  0x20
BL_PACKET_FW_UPDATE_REQ_DATA0           =  0x31
BL_PACKET_FW_UPDATE_RES_DATA0           =  0x37
BL_PACKET_DEVICE_ID_REQ_DATA0           =  0x3C
BL_PACKET_DEVICE_ID_RES_DATA0           =  0x3F
BL_PACKET_FW_LENGTH_REQ_DATA0           =  0x42
BL_PACKET_FW_LENGTH_RES_DATA0           =  0x45
BL_PACKET_READY_FOR_DATA_DATA0          =  0x48
BL_PACKET_UPDATE_SUCCESSFUL_DATA0       =  0x54
BL_PACKET_NACK_DATA0                    =  0x59
BL_PACKET_IS_CRYPTED_FW_RES_DATA0       =  0x66
BL_PACKET_IS_NOT_CRYPTED_FW_RES_DATA0   =  0x67

SERIALPATH                              = "COM14"   #DA PRA USAR ARGV ###########MUDAR PRA 14!!!!
BAUDRATE                                = 115200    #DA PRA USAR ARGV

SYNC_SEQ                                = [0xC4,0x55,0x7E,0x10]
DEFAULT_TIMEOUT                         = (5000)

BOOTLOADER_SIZE                         = (0x8000)  #DA PRA USAR ARGV
VECTOR_TABLE_SIZE                       = (0x01B0)  #DA PRA USAR ARGV

FWINFO_SENTINEL_OFFSET                  = (VECTOR_TABLE_SIZE + (0 * 4))
FWINFO_DEVICE_ID_OFFSET                 = (VECTOR_TABLE_SIZE + (1 * 4))
FWINFO_LENGTH_OFFSET                    = (VECTOR_TABLE_SIZE + (3 * 4))

def crc8(data):
    crc = 0
    for byte in data:
        crc = (crc ^ byte) & 0xff
        for i in range(8):
            if (crc & 0x80):
                crc = ((crc << 1) ^ 0x07) & 0xff
            else:
                crc = (crc << 1) & 0xff
    return crc

def delay(ms):
    time.sleep(ms/1000) # DE SEGUNDOS PRA MILLISEGUNDOS

class Logger: #CLASSE DAS MSG
  def info(message):
    print(f"[.] {message}")
  def success(message):
    print(f"[$] {message}")
  def error(message):
    print(f"[!] {message}")

class Packet:
    def __init__(self, length, data, crc=None):
        self.length = length
        self.data = data

        bytesToPad = PACKET_DATA_BYTES - len(self.data)
        for bytes in range(bytesToPad):
            self.data.append(0xff)
        if(crc is None):
            self.crc = self.computeCrc(self)
        else: 
           self.crc = crc
    def computeCrc(nada, self):
        allData = []
        allData.append(self.length)
        for byte in self.data:
            allData.append(byte)
        # print(f'alldata: {allData}')
        return crc8(allData)
    def toBuffer(nada, self):
        list = []
        list.append(self.length)
        for bytes in self.data:
            list.append(bytes)
        list.append(self.crc)
        return list
    def isSingleBytePacket(self):
        if(self.length != 1):
            return 0
        for i in range(1, PACKET_DATA_BYTES):
            if(self.data[i] != 0xff):
                return 0
        return 1
    def isThatSingleBytePacket(self, byte):
        if(self.isSingleBytePacket):
            if(self.data[0] != byte):
                return 0
            return 1
        return 0
    def isAck(self):
        return self.isThatSingleBytePacket(PACKET_ACK_DATA0)
    def isRetx(self):
        return self.isThatSingleBytePacket(PACKET_RETX_DATA0)
    def createSingleBytePacket(byte):
        return Packet(1, list.append([byte]))

RETX = Packet(1, [PACKET_RETX_DATA0])
ACK  = Packet(1, [PACKET_ACK_DATA0 ])

# RETX = RETX.toBuffer(RETX)
# ACK  = ACK.toBuffer(ACK)

uart = serial.Serial(port=SERIALPATH, baudrate=BAUDRATE)

packets = []
lastPacket = Packet(1,[PACKET_ACK_DATA0]);
lastPacket = lastPacket.toBuffer(self=lastPacket)

def writePacket(packet): # EMVIA UM PACOTE FORMATO DE ARRAY DE BYTES
    uart.write(packet)
    lastPacket = packet

rxBuffer = []

def consumeFromBuffer(n, rxBuffer):
    consumed = rxBuffer[0 : n]
    rxBuffer = rxBuffer[n:]
    return consumed

def Error(cause):
    Logger.error(f"Error - {cause}")
    uart.close()
    sys.exit(1)

# pacoteTeste = Packet(9, [1, 2, 3, 4, 5, 6, 7, 8, 9])
pacoteTeste = Packet(1, [49, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255])
pacoteTeste = pacoteTeste.toBuffer(self=pacoteTeste) # pacote para teste

# FUNÇÃO PRINCIPAL
def main():
    # 1 - CARREGAR O FIRMWARE
    with open(sys.argv[1], 'rb') as firmware:
        content = firmware.read() # FIRMWARE
    fwLength = len(content)
    bytesWritten = 0
    if uart.is_open:
        i = 0
        Logger.info('Attempting to sync with the bootloader')
        writePacket(SYNC_SEQ) # 1 - MANDAR SEQUENCIA DE SINCRONIZAÇÃO
        while uart.is_open:
            raw_data = uart.read() # 2 - RECEBER CHECKOUT DE SINCRONIZACAO 0X20 OU 32U
            #i += 1
            #print(i)
            if raw_data is not None:
                rawList = int.from_bytes(raw_data, "little")
                    #print(rawList)
                rxBuffer.append(rawList)
            while len(rxBuffer) >= PACKET_LENGTH:  # X - QUANDO PACOTE FOR FORMADO PROCESSALO CORRETAMENTE
                if lastPacket[1] != 21:
                    print(rxBuffer)
                packet = Packet(rxBuffer[0], rxBuffer[1:len(rxBuffer)-1], rxBuffer[PACKET_CRC_INDEX])
                computedCrc = packet.computeCrc(self=packet)
                delay(10)
                if(packet.crc != computedCrc): 
                    writePacket(Packet.retx)
                    print("crc != computedCrc")
                    continue
            

                if(packet.isRetx()):
                    print("Received a RETX Packet")
                    writePacket(lastPacket)
                    continue

                
                if(packet.isSingleBytePacket()):
                    state = packet.data[0]
                    # Error("BKPT1")
                    if state != 21: # SO VAI SERVIR PRA TAREFAS N REPETITIVAS
                        if state != 72:
                            print("Esse é o estado: ", state)
                            if(state == int(BL_PACKET_SYNC_OBSERVED_DATA0)):
                                Logger.success('Synced!')
                                Logger.info('Requesting firmware update...')
                                tempPacket = Packet(1, [BL_PACKET_FW_UPDATE_REQ_DATA0])
                                tempPacket = tempPacket.toBuffer(self=tempPacket)
                                writePacket(tempPacket)
                            elif(state == int(BL_PACKET_FW_UPDATE_RES_DATA0)):
                                Logger.info("Received Firmware Update Response")
                                Logger.info("Sending If it is crypted...")
                                if(int(sys.argv[2]) == 1):  #SE O CÓDIGO É CRIPTOGRAFADO
                                    tempPacket = Packet(1, [BL_PACKET_IS_CRYPTED_FW_RES_DATA0])
                                elif(int(sys.argv[2]) == 0):    #SE O CÓDIGO NÃO É CRIPTOGRAFADO
                                    tempPacket = Packet(1, [BL_PACKET_IS_NOT_CRYPTED_FW_RES_DATA0])
                                else: 
                                    Error("Invalid Argument: It is crypted?")
                                tempPacket = tempPacket.toBuffer(self=tempPacket)
                                #print("esse é o tempPacket: ", tempPacket)
                                writePacket(tempPacket)
                                #print(f"PACOTE: {tempPacket}")
                            elif(state == int(BL_PACKET_DEVICE_ID_REQ_DATA0)):
                                Logger.info("Received Device ID Request")
                                Logger.info("Sending Device ID...")
                                deviceID = content[FWINFO_DEVICE_ID_OFFSET]
                                tempPacket = Packet(2, [BL_PACKET_DEVICE_ID_RES_DATA0, deviceID])
                                tempPacket = tempPacket.toBuffer(self=tempPacket)
                                writePacket(tempPacket)
                                #uart.close()
                            elif(state == int(BL_PACKET_FW_LENGTH_REQ_DATA0)):
                                Logger.info("Received Firmware Length Request")
                                Logger.info("Sending Firmware Length...")
                                fwLengthList = bytearray(5)
                                fwLengthList[0] = BL_PACKET_FW_LENGTH_RES_DATA0
                                struct.pack_into("<I", fwLengthList, 1, fwLength)
                                fwLengthPacket = Packet(5, fwLengthList)
                                fwLengthPacket = fwLengthPacket.toBuffer(self=fwLengthPacket)
                                writePacket(fwLengthPacket)
                                #DEIXAR UM TEMPO PARA APAGAR O DISCO
                                Logger.info('Waiting for a few seconds for main application to be erased...')
                                delay(1000);
                                Logger.info('Waiting for a few seconds for main application to be erased...')
                                delay(1000);
                                Logger.info('Waiting for a few seconds for main application to be erased...')
                                delay(1000);    
                            elif(state == int(BL_PACKET_UPDATE_SUCCESSFUL_DATA0)):
                                Logger.success("Firmware update complete!")
                                uart.close()
                            else:
                                print("nada")
                        elif state == 72 or lastPacket == 72:
                            while(bytesWritten < fwLength):
                                if state == 72:
                                    dataBytes = content[bytesWritten:bytesWritten+len(packet.data)]
                                    if(fwLength-bytesWritten < 16):
                                        dataBytes = dataBytes.ljust(PACKET_DATA_BYTES, b'\xff')
                                    dataLength = len(packet.data)
                                    dataPacket = Packet(dataLength - 1, dataBytes)
                                    dataPacket = dataPacket.toBuffer(self=dataPacket)
                                    writePacket(dataPacket)
                                    bytesWritten += dataLength
                                    Logger.info(f'Wrote {dataLength} bytes ({bytesWritten}/{fwLength})')
                                    delay(10)
                if(packet.isThatSingleBytePacket(BL_PACKET_NACK_DATA0)):
                    Error('Received NACK. Exiting...')
                rxBuffer.clear()
                #print(len(rxBuffer))
                #uart.close()
            #delay(1000)
            #print("platano")


main()
delay(1000)

#Logger.info(pacoteTeste.toBuffer(self=pacoteTeste))
# Logger.info(f'{lastPacket.toBuffer(self=lastPacket)}')
# Logger.info(f'{packet.toBuffer(self=packet)}')
# Logger.info(f'[{ack.length},{ack.data},{ack.crc}]')