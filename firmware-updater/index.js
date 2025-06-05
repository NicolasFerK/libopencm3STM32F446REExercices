"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
const serialport_1 = require("serialport");
const fs = __importStar(require("fs/promises"));
const path = __importStar(require("path"));
const PACKET_LENGTH_BYTES = 1;
const PACKET_DATA_BYTES = 16;
const PACKET_CRC_BYTES = 1;
const PACKET_CRC_INDEX = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES;
const PACKET_LENGTH = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES + PACKET_CRC_BYTES;
const PACKET_ACK_DATA0 = 0x15;
const PACKET_RETX_DATA0 = 0x19;
const BL_PACKET_SYNC_OBSERVED_DATA0 = 0x20;
const BL_PACKET_FW_UPDATE_REQ_DATA0 = 0x31;
const BL_PACKET_FW_UPDATE_RES_DATA0 = 0x37;
const BL_PACKET_DEVICE_ID_REQ_DATA0 = 0x3C;
const BL_PACKET_DEVICE_ID_RES_DATA0 = 0x3F;
const BL_PACKET_FW_LENGTH_REQ_DATA0 = 0x42;
const BL_PACKET_FW_LENGTH_RES_DATA0 = 0x45;
const BL_PACKET_READY_FOR_DATA_DATA0 = 0x48;
const BL_PACKET_UPDATE_SUCCESSFUL_DATA0 = 0x54;
const BL_PACKET_NACK_DATA0 = 0x59;
const serialPath = "COM14";
const baudRate = 115200;
const SYNC_SEQ = Buffer.from([0xC4, 0x55, 0x7E, 0x10]);
const DEFAULT_TIMEOUT = (5000);
const BOOTLOADER_SIZE = (0x8000);
const VECTOR_TABLE_SIZE = (0x01B0);
const FWINFO_SENTINEL_OFFSET = (VECTOR_TABLE_SIZE + (0 * 4));
const FWINFO_DEVICE_ID_OFFSET = (VECTOR_TABLE_SIZE + (1 * 4));
const FWINFO_LENGTH_OFFSET = (VECTOR_TABLE_SIZE + (3 * 4));
const crc8 = (data) => {
    let crc = 0;
    for (const byte of data) {
        crc = (crc ^ byte) & 0xff;
        for (let i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = ((crc << 1) ^ 0x07) & 0xff;
            }
            else {
                crc = (crc << 1) & 0xff;
            }
        }
    }
    return crc;
};
const crc32 = (data, length) => {
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
const delay = (ms) => new Promise(r => setTimeout(r, ms));
class Logger {
    static info(message) { console.log(`[.] ${message}`); }
    static success(message) { console.log(`[$] ${message}`); }
    static error(message) { console.log(`[!] ${message}`); }
}
class Packet {
    constructor(length, data, crc) {
        this.length = length;
        this.data = data;
        const bytesToPad = PACKET_DATA_BYTES - this.data.length;
        const padding = Buffer.alloc(bytesToPad).fill(0xff);
        this.data = Buffer.concat([this.data, padding]);
        if (typeof crc === 'undefined') {
            this.crc = this.computeCrc();
        }
        else {
            this.crc = crc;
        }
    }
    computeCrc() {
        const allData = [this.length, ...this.data];
        return crc8(allData);
    }
    toBuffer() {
        return Buffer.concat([Buffer.from([this.length]), this.data, Buffer.from([this.crc])]);
    }
    isSingleBytePacket(byte) {
        if (this.length !== 1)
            return false;
        if (this.data[0] !== byte)
            return false;
        for (let i = 1; i < PACKET_DATA_BYTES; i++) {
            if (this.data[i] !== 0xff)
                return false;
        }
        return true;
    }
    isAck() {
        return this.isSingleBytePacket(PACKET_ACK_DATA0);
    }
    isRetx() {
        return this.isSingleBytePacket(PACKET_RETX_DATA0);
    }
    static createSingleBytePacket(byte) {
        return new Packet(1, Buffer.from([byte]));
    }
}
Packet.retx = new Packet(1, Buffer.from([PACKET_RETX_DATA0])).toBuffer();
Packet.ack = new Packet(1, Buffer.from([PACKET_ACK_DATA0])).toBuffer();
const uart = new serialport_1.SerialPort({ path: serialPath, baudRate });
let packets = [];
let lastPacket = Packet.ack;
const writePacket = (packet) => {
    uart.write(packet);
    lastPacket = packet;
};
let rxBuffer = Buffer.from([]);
const consumeFromBuffer = (n) => {
    const consumed = rxBuffer.slice(0, n);
    rxBuffer = rxBuffer.slice(n);
    return consumed;
};
const pacoteTeste = new Packet(9, Buffer.from([1, 2, 3, 4, 5, 6, 7, 8, 9, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]), 127);
uart.on('data', data => {
    rxBuffer = Buffer.concat([rxBuffer, data]);
    while (rxBuffer.length >= PACKET_LENGTH) { //ver esse bloco
        const raw = consumeFromBuffer(PACKET_LENGTH);
        const packet = new Packet(raw[0], raw.slice(1, 1 + PACKET_DATA_BYTES), raw[PACKET_CRC_INDEX]);
        const computedCrc = packet.computeCrc();
        //console.log(computedCrc + " " + packet.crc);
        if (packet.crc !== computedCrc) {
            writePacket(Packet.retx);
            continue;
        }
        if (packet.isRetx()) {
            writePacket(lastPacket);
            continue;
        }
        if (packet.isAck()) {
            continue;
        }
        if (packet.isSingleBytePacket(BL_PACKET_NACK_DATA0)) {
            Logger.error('Received NACK. Exiting...');
            process.exit(1);
        }
        packets.push(packet);
        writePacket(Packet.ack);
    }
});
const waitForPacket = (...args_1) => __awaiter(void 0, [...args_1], void 0, function* (timeout = DEFAULT_TIMEOUT) {
    let timeWaited = 0;
    while (packets.length < 1) {
        yield delay(1);
        timeWaited += 1;
        if (timeWaited >= timeout) {
            throw Error('Timed out waiting for a packet');
        }
    }
    return packets.splice(0, 1)[0];
});
const waitForSingleBytePacket = (byte_1, ...args_1) => __awaiter(void 0, [byte_1, ...args_1], void 0, function* (byte, timeout = DEFAULT_TIMEOUT) {
    try {
        const packet = yield waitForPacket(timeout);
        if (packet.length !== 1 || packet.data[0] !== byte) {
            throw new Error(`Unexpected packet received. Expected single byte 0x${byte.toString(16)}`);
        }
    }
    catch (e) {
        Logger.error(e.message);
        console.log(rxBuffer);
        console.log(packets);
        process.exit(1);
    }
});
const syncWithBootloader = (...args_1) => __awaiter(void 0, [...args_1], void 0, function* (timeout = DEFAULT_TIMEOUT) {
    let timeWaited = 0;
    while (true) {
        uart.write(SYNC_SEQ);
        yield delay(1000);
        timeWaited += 1000;
        if (packets.length > 0) {
            const packet = packets.splice(0, 1)[0];
            if (packet.isSingleBytePacket(BL_PACKET_SYNC_OBSERVED_DATA0)) {
                return;
            }
            Logger.error('Wrong packet observed during sync sequence');
            process.exit(1);
        }
        if (timeWaited >= timeout) {
            Logger.error('Timed out waiting for a sync sequence observed');
            process.exit(1);
        }
    }
});
const main = () => __awaiter(void 0, void 0, void 0, function* () {
    if (process.argv.length < 3) {
        console.log("usage: fw-updater <signed firmware>");
        process.exit(1);
    }
    const firmwareFilename = process.argv[2];
    Logger.info(`Reading the firmware image...`);
    const fwImage = yield fs.readFile(path.join(process.cwd(), firmwareFilename));
    const fwLength = fwImage.length;
    Logger.success(`Read firmware image (${fwLength} bytes)`);
    Logger.info(`${path.join(process.cwd(), 'firmware.bin')}`);
    Logger.info('Attempting to sync with the bootloader');
    yield syncWithBootloader();
    Logger.success('Synced!');
    Logger.info('Requesting firmware update');
    const fwUpdatePacket = Packet.createSingleBytePacket(BL_PACKET_FW_UPDATE_REQ_DATA0);
    writePacket(fwUpdatePacket.toBuffer());
    yield waitForSingleBytePacket(BL_PACKET_FW_UPDATE_RES_DATA0);
    Logger.success('Firmware update request accepted');
    Logger.info('Waiting for device ID request');
    yield waitForSingleBytePacket(BL_PACKET_DEVICE_ID_REQ_DATA0);
    Logger.success('Device ID request received');
    const deviceId = fwImage[FWINFO_DEVICE_ID_OFFSET]; //VER MELHOR COMO FUNCIONA ISSO
    const deviceIDPacket = new Packet(2, Buffer.from([BL_PACKET_DEVICE_ID_RES_DATA0, deviceId]));
    writePacket(deviceIDPacket.toBuffer());
    Logger.info(`Responding with device ID 0x${deviceId.toString(16)}`);
    Logger.info(`Waiting for firmware length request`);
    yield waitForSingleBytePacket(BL_PACKET_FW_LENGTH_REQ_DATA0);
    const fwLengthPacketBuffer = Buffer.alloc(5);
    fwLengthPacketBuffer[0] = BL_PACKET_FW_LENGTH_RES_DATA0;
    fwLengthPacketBuffer.writeUInt32LE(fwLength, 1);
    const fwLengthPacket = new Packet(5, fwLengthPacketBuffer);
    writePacket(fwLengthPacket.toBuffer());
    Logger.info('Responding with firmware length');
    Logger.info('Waiting for a few seconds for main application to be erased...');
    yield delay(1000);
    Logger.info('Waiting for a few seconds for main application to be erased...');
    yield delay(1000);
    Logger.info('Waiting for a few seconds for main application to be erased...');
    yield delay(1000);
    let bytesWritten = 0;
    while (bytesWritten < fwLength) {
        yield waitForSingleBytePacket(BL_PACKET_READY_FOR_DATA_DATA0);
        const dataBytes = fwImage.slice(bytesWritten, bytesWritten + PACKET_DATA_BYTES);
        const dataLength = dataBytes.length;
        const dataPacket = new Packet(dataLength - 1, dataBytes);
        writePacket(dataPacket.toBuffer());
        bytesWritten += dataLength;
        //Logger.info(`${dataLength}`);
        Logger.info(`Wrote ${dataLength} bytes (${bytesWritten}/${fwLength})`);
    }
    yield waitForSingleBytePacket(BL_PACKET_UPDATE_SUCCESSFUL_DATA0);
    Logger.success("Firmware update complete!");
});
main()
    .finally(() => uart.close());
