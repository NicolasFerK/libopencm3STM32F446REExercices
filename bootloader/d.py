while uart.is_open:                                                                         
    writePacket(pacoteTeste)
    raw_data = uart.read()
    if raw_data is not None:
        rawList = int.from_bytes(raw_data, "big")
    rxBuffer.append(rawList)
    print(rxBuffer) # mostra o buffer atual
    # print(rxBuffer.hex())
    while len(rxBuffer) >= PACKET_LENGTH:
        raw = consumeFromBuffer(PACKET_LENGTH, rxBuffer)
        packet = Packet(raw[0], raw[1:1+PACKET_DATA_BYTES], raw[PACKET_CRC_INDEX])
        #print(packet)
        computedCrc = packet.computeCrc(packet)
        #print(f'crc = {packet.crc}')
        if(packet.crc != computedCrc):
            #print(packet.crc)
            # print(computedCrc)
            retx = RETX.toBuffer(self=RETX)
            writePacket(retx)
            breakpoint(1)
            continue
        if(packet.isRetx()):
            writePacket(lastPacket)
            breakpoint(2)
            continue
                
        if(packet.isAck()):
            continue
        if(packet.isSingleBytePacket(BL_PACKET_NACK_DATA0)):
            Logger.error('Received NACK. Exiting...')
            sys.exit(1)
        packets.extend(packet)
        writePacket(Packet.ack)
        break

#FUNÇÃO 1DE4
def waitForPacket(timeout = DEFAULT_TIMEOUT):
    print("oi")
    timeWaited = 0
    while(packets.length < 1):
        delay(1)
        timeWaited += 1
        if(timeWaited >= timeout): 
            Logger.error('Timed out waiting for a packet')
            sys.exit(1)
    return packets.pop(0) # .splice(0,1)[0]

#FUNÇÃO 2DE4
def waitForSingleBytePacket(byte, timeout = DEFAULT_TIMEOUT):
    print("oi2")
    try:
        packet = waitForPacket(timeout)
        if(packet.length != 1 or packet.data[0] != byte):
            raise Exception(f'Unexpected packet received. Expected single byte 0x{byte.toString(16)}')
    except Exception as e:
        Logger.error(e.message)
        print(rxBuffer)
        print(packets)
        sys.exit(1)

#FUNÇÃO 3DE4
def syncWithBootloader(timeout = DEFAULT_TIMEOUT):
  print("oi3")
  timeWaited = 0
  while(1):
    uart.write(SYNC_SEQ)
    delay(1000)
    timeWaited += 1000
    if(packets.length > 0):
        packet = packets.splice(0, 1)[0]
        if(packet.isSingleBytePacket(BL_PACKET_SYNC_OBSERVED_DATA0)):
            return
        Logger.error('Wrong packet observed during sync sequence')
        sys.exit(1)
    if(timeWaited >= timeout):
        Logger.error('Timed out waiting for a sync sequence observed')
        sys.exit(1)

#FUNÇÃO 4DE4
def main():
    if(sys.argv.length < 4):
        print("usage: fw-updater <signed firmware>")
        sys.exit(1)
  
    firmwareFilename = sys.argv[2]
    IsFirmwareCrypted = int(sys.argv[3], 2) #segundo argumento do comando vai ser 0 ou 1, 0 pra codigo normal e 1 pra codigo criptografado

    Logger.info('Reading the firmware image...')
    with open(firmwareFilename, "rb") as f:
        fwImage = f.read()
        f.close()
    fwLength = fwImage.length
