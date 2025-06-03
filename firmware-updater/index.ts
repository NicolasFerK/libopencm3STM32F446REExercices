import { time } from 'console';
import { SerialPort } from 'serialport';
import * as fs from 'fs/promises';
import * as path from 'path';

const PACKET_LENGTH_BYTES                  = 1;
const PACKET_DATA_BYTES                    = 16;
const PACKET_CRC_BYTES                     = 1;
const PACKET_CRC_INDEX                     = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES;
const PACKET_LENGTH                        = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES + PACKET_CRC_BYTES;

const PACKET_ACK_DATA0                     = 0x15; 
const PACKET_RETX_DATA0                    = 0x19;

const BL_PACKET_SYNC_OBSERVED_DATA0        =  0x20;
const BL_PACKET_FW_UPDATE_REQ_DATA0        =  0x31;
const BL_PACKET_FW_UPDATE_RES_DATA0        =  0x37;
const BL_PACKET_DEVICE_ID_REQ_DATA0        =  0x3C;
const BL_PACKET_DEVICE_ID_RES_DATA0        =  0x3F;
const BL_PACKET_FW_LENGTH_REQ_DATA0        =  0x42;
const BL_PACKET_FW_LENGTH_RES_DATA0        =  0x45;
const BL_PACKET_READY_FOR_DATA_DATA0       =  0x48;
const BL_PACKET_UPDATE_SUCCESSFUL_DATA0     =  0x54;
const BL_PACKET_NACK_DATA0                 =  0x59;

const serialPath                           = "COM14";
const baudRate                             = 115200;

const SYNC_SEQ = Buffer.from([0xC4,0x55,0x7E,0x10]);
const DEFAULT_TIMEOUT = (5000);

const BOOTLOADER_SIZE                   = (0x8000);
const VECTOR_TABLE_SIZE                 = (0x1AC);
const FIRMWARE_INFO_SIZE                = (10 * 4);

const FWINFO_VALIDATE_FROM              = (VECTOR_TABLE_SIZE + FIRMWARE_INFO_SIZE);
const FWINFO_SENTINEL_OFFSET            = (VECTOR_TABLE_SIZE + (0 * 4));
const FWINFO_DEVICE_ID_OFFSET           = (VECTOR_TABLE_SIZE + (1 * 4));
const FWINFO_VERSION_OFFSET             = (VECTOR_TABLE_SIZE + (2 * 4));
const FWINFO_LENGTH_OFFSET              = (VECTOR_TABLE_SIZE + (3 * 4));
const FWINFO_CRC32_OFFSET               = (VECTOR_TABLE_SIZE + (9 * 4));

const crc8 = (data: Buffer | Array<number>) => 
  {
    let crc = 0;

    for (const byte of data) {
        crc = (crc ^ byte) & 0xff;
        for (let i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = ((crc << 1) ^ 0x07) & 0xff;
            } else {
                crc = (crc << 1) & 0xff;
            }
        }
    }
    return crc;
}; 
const crc32 = (data: Buffer, length: number) => {
  let byte;
  let crc = 0xffffffff;
  let mask;

  for (let i = 0; i < length; i++) {
     byte = data[i];
     crc = (crc ^ byte) >>> 0;

     for (let j = 0; j < 8; j++) {
        mask = (-(crc & 1)) >>> 0;
        crc = ((crc >>> 1) ^ (0xedb88320 & mask)) >>> 0;
     }
  }

  return (~crc) >>> 0;
};
const delay = (ms: number) => new Promise(r => setTimeout(r, ms));

class Logger {
  static info(message: string) {console.log(`[.] ${message}`);}
  static success(message: string) {console.log(`[$] ${message}`);}
  static error(message: string) {console.log(`[!] ${message}`);}

}

class Packet {
length: number;
data: Buffer;
crc: number;

static retx = new Packet(1, Buffer.from([PACKET_RETX_DATA0])).toBuffer();
static ack = new Packet(1, Buffer.from([PACKET_ACK_DATA0])).toBuffer();

constructor(length: number, data: Buffer, crc?: number) {
  this.length = length;
  this.data = data;

  const bytesToPad = PACKET_DATA_BYTES - this.data.length;
  const padding = Buffer.alloc(bytesToPad).fill(0xff);
  this.data = Buffer.concat([this.data, padding]);

  if(typeof crc === 'undefined') {
    this.crc = this.computeCrc();
  } else {
    this.crc = crc;
  }
}

computeCrc() {
  const allData = [this.length, ...this.data];
  return crc8(allData);
}

toBuffer() {
  return Buffer.concat([ Buffer.from([this.length]), this.data, Buffer.from([this.crc]) ]);
}

isSingleBytePacket(byte: number) {
  if(this.length !== 1) return false;
  if(this.data[0] !== byte) return false;
  for (let i = 1; i < PACKET_DATA_BYTES; i++) {
    if(this.data[i] !== 0xff) return false;
  }
  return true;
}

isAck() {
  return this.isSingleBytePacket(PACKET_ACK_DATA0);
}

isRetx() {
  return this.isSingleBytePacket(PACKET_RETX_DATA0);
}

static createSingleBytePacket(byte: number) {
  return new Packet(1, Buffer.from([byte]));
}

}

const uart = new SerialPort({ path: serialPath, baudRate });

let packets: Packet[] = [];

let lastPacket: Buffer = Packet.ack;
const writePacket = (packet: Buffer) => {
  uart.write(packet);
  lastPacket = packet; 
}

let rxBuffer = Buffer.from([]);
const consumeFromBuffer = (n: number) => {
  const consumed = rxBuffer.slice(0, n);
  rxBuffer = rxBuffer.slice(n);
  return consumed;
}

const pacoteTeste = new Packet(
        9,
        Buffer.from([1, 2, 3, 4, 5, 6, 7, 8, 9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]),
        127
);

uart.on('data', data => {
  rxBuffer = Buffer.concat([rxBuffer, data]);

  while (rxBuffer.length >= PACKET_LENGTH) {  //ver esse bloco
    const raw = consumeFromBuffer(PACKET_LENGTH);
    const packet = new Packet(raw[0], raw.slice(1, 1+PACKET_DATA_BYTES), raw[PACKET_CRC_INDEX]);
    const computedCrc = packet.computeCrc();
    //console.log(computedCrc + " " + packet.crc);
    if(packet.crc !== computedCrc) {
      writePacket(Packet.retx);
      continue;
    }

    if(packet.isRetx()){
      writePacket(lastPacket);
      continue;
    }    

    if(packet.isAck()){
      continue;
    }

    if(packet.isSingleBytePacket(BL_PACKET_NACK_DATA0)){
      Logger.error('Received NACK. Exiting...');
      process.exit(1);
    }

    packets.push(packet);
    writePacket(Packet.ack);
  }
});

const waitForPacket = async (timeout = DEFAULT_TIMEOUT) => {
  let timeWaited = 0;
  while(packets.length < 1) {
    await delay(1);
    timeWaited += 1;
    if(timeWaited >= timeout) {
      throw Error('Timed out waiting for a packet');
    }
  }
  return packets.splice(0,1)[0];
}

const waitForSingleBytePacket = async (byte: number, timeout = DEFAULT_TIMEOUT) => {
  try {
    const packet = await waitForPacket(timeout);
    if (packet.length !== 1 || packet.data[0] !== byte) {
      throw new Error(`Unexpected packet received. Expected single byte 0x${byte.toString(16)}`);
    }
  } catch (e: any) {
    Logger.error(e.message);
    console.log(rxBuffer);
    console.log(packets);
    process.exit(1);
  }
};


const syncWithBootloader = async (timeout = DEFAULT_TIMEOUT) => {
  let timeWaited = 0;
  while(true){
    uart.write(SYNC_SEQ);
    await delay(1000);
    timeWaited += 1000;
    if(packets.length > 0) {
      const packet = packets.splice(0, 1)[0];
      if(packet.isSingleBytePacket(BL_PACKET_SYNC_OBSERVED_DATA0)) {
        return;
      }
      Logger.error('Wrong packet observed during sync sequence');
      process.exit(1);
    }
    if(timeWaited >= timeout)
    {
      Logger.error('Timed out waiting for a sync sequence observed');
      process.exit(1);
    }
  }
}

const main = async () => {
  Logger.info(`Reading the firmware image...`);
  const fwImage = await fs.readFile(path.join(process.cwd(), 'firmware.bin'))
  .then(bin => bin.slice(BOOTLOADER_SIZE));
  const fwLength = fwImage.length;
  Logger.success(`Read firmware image (${fwLength} bytes)`);

  Logger.success(`Injecting into firmware information section`);
  fwImage.writeUInt32LE(fwLength, FWINFO_LENGTH_OFFSET);
  fwImage.writeUInt32LE(0x00000001, FWINFO_VERSION_OFFSET); //VERSAO 1
  
  const crcValue = crc32(fwImage.slice(FWINFO_VALIDATE_FROM), fwLength - (VECTOR_TABLE_SIZE + FIRMWARE_INFO_SIZE));
  Logger.info(`Computed CRC value: 0x${crcValue.toString(16).padStart(8, '0')}`);
  fwImage.writeUInt32LE(crcValue, FWINFO_CRC32_OFFSET);

  Logger.info('Attempting to sync with the bootloader');
  await syncWithBootloader();
  Logger.success('Synced!');

  Logger.info('Requesting firmware update');
  const fwUpdatePacket = Packet.createSingleBytePacket(BL_PACKET_FW_UPDATE_REQ_DATA0);
  writePacket(fwUpdatePacket.toBuffer());
  
  await waitForSingleBytePacket(BL_PACKET_FW_UPDATE_RES_DATA0);
  Logger.success('Firmware update request accepted');
  
  Logger.info('Waiting for device ID request');
  await waitForSingleBytePacket(BL_PACKET_DEVICE_ID_REQ_DATA0);
  Logger.success('Device ID request received');

  const deviceId = fwImage[FWINFO_DEVICE_ID_OFFSET];  //VER MELHOR COMO FUNCIONA ISSO
  const deviceIDPacket = new Packet(2, Buffer.from([BL_PACKET_DEVICE_ID_RES_DATA0, deviceId]));
  writePacket(deviceIDPacket.toBuffer());
  Logger.info(`Responding with device ID 0x${deviceId.toString(16)}`);

  Logger.info(`Waiting for firmware length request`);
  await waitForSingleBytePacket(BL_PACKET_FW_LENGTH_REQ_DATA0);

  const fwLengthPacketBuffer = Buffer.alloc(5);
  fwLengthPacketBuffer[0] = BL_PACKET_FW_LENGTH_RES_DATA0;
  fwLengthPacketBuffer.writeUInt32LE(fwLength, 1);
  const fwLengthPacket = new Packet(5, fwLengthPacketBuffer);
  writePacket(fwLengthPacket.toBuffer());
  Logger.info('Responding with firmware length');
  
  Logger.info('Waiting for a few seconds for main application to be erased...');
  await delay(1000);
  Logger.info('Waiting for a few seconds for main application to be erased...');
  await delay(1000);
  Logger.info('Waiting for a few seconds for main application to be erased...');
  await delay(1000);

  let bytesWritten = 0;
 while (bytesWritten < fwLength) {
    await waitForSingleBytePacket(BL_PACKET_READY_FOR_DATA_DATA0);

    const dataBytes = fwImage.slice(bytesWritten, bytesWritten + PACKET_DATA_BYTES);
    const dataLength = dataBytes.length;
    const dataPacket = new Packet(dataLength - 1, dataBytes);
    writePacket(dataPacket.toBuffer());
    bytesWritten += dataLength;
    //Logger.info(`${dataLength}`);
    Logger.info(`Wrote ${dataLength} bytes (${bytesWritten}/${fwLength})`);
  }

  await waitForSingleBytePacket(BL_PACKET_UPDATE_SUCCESSFUL_DATA0);
  Logger.success("Firmware update complete!");
} 

main()
  .finally(() => uart.close());