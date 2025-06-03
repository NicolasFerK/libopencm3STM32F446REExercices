"use strict";
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
const PACKET_LENGTH_BYTES = 1;
const PACKET_DATA_BYTES = 16;
const PACKET_CRC_BYTES = 1;
const PACKET_CRC_INDEX = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES;
const PACKET_LENGTH = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES + PACKET_CRC_BYTES;
const PACKET_ACK_DATA0 = 0x15;
const PACKET_RETX_DATA0 = 0x19;
const serialPath = "COM20";
const baudRate = 115200;
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
const delay = (ms) => new Promise(r => setTimeout(r, ms));
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
uart.on('data', data => {
    console.log(`Received ${data.length} bytes through uart`);
    rxBuffer = Buffer.concat([rxBuffer, data]);
    if (rxBuffer.length >= PACKET_LENGTH) {
        console.log(`Building a packet`);
        const raw = consumeFromBuffer(PACKET_LENGTH);
        const packet = new Packet(raw[0], raw.slice(1, 1 + PACKET_DATA_BYTES), raw[PACKET_CRC_INDEX]);
        const computedCrc = packet.computeCrc();
        if (packet.crc !== computedCrc) {
            console.log(`CRC failed, computed 0x${computedCrc.toString(16)}, got 0x${packet.crc.toString(16)}`);
            writePacket(Packet.retx);
            return;
        }
        if (packet.isRetx()) {
            console.log(`Retransmitting last packet`);
            writePacket(lastPacket);
            return;
        }
        if (packet.isAck()) {
            console.log(`It was an ack, nothing to do`);
            return;
        }
        console.log(`Storing packet and ack'ing`);
        packets.push(packet);
        writePacket(Packet.ack);
    }
});
const waitForPacket = () => __awaiter(void 0, void 0, void 0, function* () {
    while (packets.length < 1) {
        yield delay(1);
    }
    const packet = packets[0];
    packets = packets.slice[1];
    return packet;
});
const main = () => __awaiter(void 0, void 0, void 0, function* () {
    console.log('Waiting for packet...');
    const packet = yield waitForPacket();
    console.log(packet);
});
main();
