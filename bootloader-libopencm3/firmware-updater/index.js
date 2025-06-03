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
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g = Object.create((typeof Iterator === "function" ? Iterator : Object).prototype);
    return g.next = verb(0), g["throw"] = verb(1), g["return"] = verb(2), typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (g && (g = 0, op[0] && (_ = 0)), _) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
var __spreadArray = (this && this.__spreadArray) || function (to, from, pack) {
    if (pack || arguments.length === 2) for (var i = 0, l = from.length, ar; i < l; i++) {
        if (ar || !(i in from)) {
            if (!ar) ar = Array.prototype.slice.call(from, 0, i);
            ar[i] = from[i];
        }
    }
    return to.concat(ar || Array.prototype.slice.call(from));
};
Object.defineProperty(exports, "__esModule", { value: true });
var fs = require("fs/promises");
var path = require("path");
var serialport_1 = require("serialport");
// Constants for the packet protocol
var PACKET_LENGTH_BYTES = 1;
var PACKET_DATA_BYTES = 16;
var PACKET_CRC_BYTES = 1;
var PACKET_CRC_INDEX = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES;
var PACKET_LENGTH = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES + PACKET_CRC_BYTES;
var PACKET_ACK_DATA0 = 0x15;
var PACKET_RETX_DATA0 = 0x19;
var BL_PACKET_SYNC_OBSERVED_DATA0 = (0x20);
var BL_PACKET_FW_UPDATE_REQ_DATA0 = (0x31);
var BL_PACKET_FW_UPDATE_RES_DATA0 = (0x37);
var BL_PACKET_DEVICE_ID_REQ_DATA0 = (0x3C);
var BL_PACKET_DEVICE_ID_RES_DATA0 = (0x3F);
var BL_PACKET_FW_LENGTH_REQ_DATA0 = (0x42);
var BL_PACKET_FW_LENGTH_RES_DATA0 = (0x45);
var BL_PACKET_READY_FOR_DATA_DATA0 = (0x48);
var BL_PACKET_UPDATE_SUCCESSFUL_DATA0 = (0x54);
var BL_PACKET_NACK_DATA0 = (0x59);
var VECTOR_TABLE_SIZE = (0x01B0);
var FWINFO_DEVICE_ID_OFFSET = (VECTOR_TABLE_SIZE + (1 * 4));
var FWINFO_LENGTH_OFFSET = (VECTOR_TABLE_SIZE + (3 * 4));
var SYNC_SEQ = Buffer.from([0xc4, 0x55, 0x7e, 0x10]);
var DEFAULT_TIMEOUT = (5000);
// Details about the serial port connection
var serialPath = "COM20";
var baudRate = 115200;
// CRC8 implementation
var crc8 = function (data) {
    var crc = 0;
    for (var _i = 0, data_1 = data; _i < data_1.length; _i++) {
        var byte = data_1[_i];
        crc = (crc ^ byte) & 0xff;
        for (var i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = ((crc << 1) ^ 0x07) & 0xff;
            }
            else {
                crc = (crc << 1) & 0xff;
            }
        }
        //console.log(`0x${byte.toString(16)} 0x${crc.toString(16)}`)
    }
    return crc;
};
var crc32 = function (data, length) {
    var byte;
    var crc = 0xffffffff;
    var mask;
    for (var i = 0; i < length; i++) {
        byte = data[i];
        crc = (crc ^ byte) >>> 0;
        for (var j = 0; j < 8; j++) {
            mask = (-(crc & 1)) >>> 0;
            crc = ((crc >>> 1) ^ (0xedb88320 & mask)) >>> 0;
        }
    }
    return (~crc) >>> 0;
};
// Async delay function, which gives the event loop time to process outside input
var delay = function (ms) { return new Promise(function (r) { return setTimeout(r, ms); }); };
var Logger = /** @class */ (function () {
    function Logger() {
    }
    Logger.info = function (message) { console.log("[.] ".concat(message)); };
    Logger.success = function (message) { console.log("[$] ".concat(message)); };
    Logger.error = function (message) { console.log("[!] ".concat(message)); };
    return Logger;
}());
// Class for serialising and deserialising packets
var Packet = /** @class */ (function () {
    function Packet(length, data, crc) {
        this.length = length;
        this.data = data;
        var bytesToPad = PACKET_DATA_BYTES - this.data.length;
        var padding = Buffer.alloc(bytesToPad).fill(0xff);
        this.data = Buffer.concat([this.data, padding]);
        if (typeof crc === 'undefined') {
            console.log("ola\n");
            this.crc = this.computeCrc();
        }
        else {
            this.crc = crc;
        }
    }
    Packet.prototype.computeCrc = function () {
        var allData = __spreadArray([this.length], this.data, true);
        return crc8(allData);
    };
    Packet.prototype.toBuffer = function () {
        return Buffer.concat([Buffer.from([this.length]), this.data, Buffer.from([this.crc])]);
    };
    Packet.prototype.isSingleBytePacket = function (byte) {
        if (this.length !== 1)
            return false;
        if (this.data[0] !== byte)
            return false;
        for (var i = 1; i < PACKET_DATA_BYTES; i++) {
            if (this.data[i] !== 0xff)
                return false;
        }
        return true;
    };
    Packet.prototype.isAck = function () {
        return this.isSingleBytePacket(PACKET_ACK_DATA0);
    };
    Packet.prototype.isRetx = function () {
        return this.isSingleBytePacket(PACKET_RETX_DATA0);
    };
    Packet.createSingleBytePacket = function (byte) {
        return new Packet(1, Buffer.from([byte]));
    };
    Packet.retx = new Packet(1, Buffer.from([PACKET_RETX_DATA0]));
    Packet.ack = new Packet(1, Buffer.from([PACKET_ACK_DATA0]));
    return Packet;
}());
// Serial port instance
var uart = new serialport_1.SerialPort({ path: serialPath, baudRate: baudRate });
// Packet buffer
var packets = [];
var lastPacket = new Packet(1, Buffer.from([0xff]));
var writePacket = function (packet) {
    uart.write(packet.toBuffer());
    lastPacket = packet;
};
// Serial data buffer, with a splice-like function for consuming data
var rxBuffer = Buffer.from([]);
var consumeFromBuffer = function (n) {
    var consumed = rxBuffer.slice(0, n);
    rxBuffer = rxBuffer.slice(n);
    return consumed;
};
// This function fires whenever data is received over the serial port. The whole
// packet state machine runs here.
uart.on('data', function (data) {
    // Add the data to the packet
    console.log("Received ".concat(data.length, " bytes through UART"));
    rxBuffer = Buffer.concat([rxBuffer, data]);
    // Can we build a packet?
    while (rxBuffer.length >= PACKET_LENGTH) {
        var raw = consumeFromBuffer(PACKET_LENGTH);
        // console.log(raw);
        var packet = new Packet(raw[0], raw.slice(1, 1 + PACKET_DATA_BYTES), raw[PACKET_CRC_INDEX]);
        var computedCrc = packet.computeCrc();
        // Need retransmission?
        if (packet.crc !== computedCrc) {
            console.log("CRC failed, computed 0x".concat(computedCrc.toString(16), ", got 0x").concat(packet.crc.toString(16)));
            writePacket(Packet.retx);
            continue;
        }
        // Are we being asked to retransmit?
        if (packet.isRetx()) {
            console.log("Retransmitting last packet");
            console.log("Last packet:", lastPacket);
            writePacket(lastPacket);
            continue;
        }
        // If this is an ack, move on
        if (packet.isAck()) {
            console.log("It was an Ack nothing to do");
            continue;
        }
        // If this is an nack, exit the program
        if (packet.isSingleBytePacket(BL_PACKET_NACK_DATA0)) {
            Logger.error('Received NACK. Exiting...');
            // console.log('packets', packets);
            // console.log('uart buffer', rxBuffer);
            process.exit(1);
        }
        // Otherwise write the packet in to the buffer, and send an ack
        console.log("Storing packet and ack'ing");
        packets.push(packet);
        writePacket(Packet.ack);
    }
});
// Function to allow us to await a packet
var waitForPacket = function () {
    var args_1 = [];
    for (var _i = 0; _i < arguments.length; _i++) {
        args_1[_i] = arguments[_i];
    }
    return __awaiter(void 0, __spreadArray([], args_1, true), void 0, function (timeout) {
        var timeWaited;
        if (timeout === void 0) { timeout = DEFAULT_TIMEOUT; }
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    timeWaited = 0;
                    _a.label = 1;
                case 1:
                    if (!(packets.length < 1)) return [3 /*break*/, 3];
                    return [4 /*yield*/, delay(1)];
                case 2:
                    _a.sent();
                    timeWaited += 1;
                    if (timeWaited >= timeout) {
                        throw Error('Timed out waiting for packet');
                    }
                    return [3 /*break*/, 1];
                case 3: return [2 /*return*/, packets.splice(0, 1)[0]];
            }
        });
    });
};
var waitForSingleBytePacket = function (byte, timeout) {
    if (timeout === void 0) { timeout = DEFAULT_TIMEOUT; }
    return (waitForPacket(timeout)
        .then(function (packet) {
        if (packet.length !== 1 || packet.data[0] !== byte) {
            var formattedPacket = __spreadArray([], packet.toBuffer(), true).map(function (x) { return x.toString(16); }).join(' ');
            throw new Error("Unexpected packet received. Expected single byte 0x".concat(byte.toString(16), "), got packet ").concat(formattedPacket));
        }
    })
        .catch(function (e) {
        Logger.error(e.message);
        console.log(rxBuffer);
        console.log(packets);
        process.exit(1);
    }));
};
var syncWithBootloader = function () {
    var args_1 = [];
    for (var _i = 0; _i < arguments.length; _i++) {
        args_1[_i] = arguments[_i];
    }
    return __awaiter(void 0, __spreadArray([], args_1, true), void 0, function (syncDelay, timeout) {
        var timeWaited, packet;
        if (syncDelay === void 0) { syncDelay = 500; }
        if (timeout === void 0) { timeout = DEFAULT_TIMEOUT; }
        return __generator(this, function (_a) {
            switch (_a.label) {
                case 0:
                    timeWaited = 0;
                    _a.label = 1;
                case 1:
                    if (!true) return [3 /*break*/, 3];
                    uart.write(SYNC_SEQ);
                    return [4 /*yield*/, delay(syncDelay)];
                case 2:
                    _a.sent();
                    timeWaited += syncDelay;
                    if (packets.length > 0) {
                        packet = packets.splice(0, 1)[0];
                        if (packet.isSingleBytePacket(BL_PACKET_SYNC_OBSERVED_DATA0)) {
                            return [2 /*return*/];
                        }
                        Logger.error('Wrong packet observed during sync sequence');
                        process.exit(1);
                    }
                    if (timeWaited >= timeout) {
                        Logger.error('Timed out waiting for sync sequence observed');
                        process.exit(1);
                    }
                    return [3 /*break*/, 1];
                case 3: return [2 /*return*/];
            }
        });
    });
};
// Do everything in an async function so we can have loops, awaits etc
var main = function () { return __awaiter(void 0, void 0, void 0, function () {
    var firmwareFilename, fwImage, fwLength, fwUpdatePacket, deviceId, deviceIDPacket, fwLengthPacketBuffer, fwLengthPacket, bytesWritten, dataBytes, dataLength, dataPacket;
    return __generator(this, function (_a) {
        switch (_a.label) {
            case 0:
                if (process.argv.length < 3) {
                    console.log("usage: fw-updater <signed firmware>");
                    process.exit(1);
                }
                firmwareFilename = process.argv[2];
                Logger.info('Reading the firmware image...');
                return [4 /*yield*/, fs.readFile(path.join(process.cwd(), firmwareFilename))];
            case 1:
                fwImage = _a.sent();
                fwLength = fwImage.length;
                Logger.success("Read firmware image (".concat(fwLength, " bytes)"));
                Logger.info('Attempting to sync with the bootloader');
                return [4 /*yield*/, syncWithBootloader()];
            case 2:
                _a.sent();
                Logger.success('Synced!');
                Logger.info('Requesting firmware update');
                fwUpdatePacket = Packet.createSingleBytePacket(BL_PACKET_FW_UPDATE_REQ_DATA0);
                writePacket(fwUpdatePacket);
                return [4 /*yield*/, waitForSingleBytePacket(BL_PACKET_FW_UPDATE_RES_DATA0)];
            case 3:
                _a.sent();
                Logger.success('Firmware update request accepted');
                Logger.info('Waiting for device ID request');
                return [4 /*yield*/, waitForSingleBytePacket(BL_PACKET_DEVICE_ID_REQ_DATA0)];
            case 4:
                _a.sent();
                Logger.success('Device ID request recieved');
                deviceId = fwImage[FWINFO_DEVICE_ID_OFFSET];
                deviceIDPacket = new Packet(2, Buffer.from([BL_PACKET_DEVICE_ID_RES_DATA0, deviceId]));
                writePacket(deviceIDPacket);
                Logger.info("Responding with device ID 0x".concat(deviceId.toString(16)));
                Logger.info('Waiting for firmware length request');
                return [4 /*yield*/, waitForSingleBytePacket(BL_PACKET_FW_LENGTH_REQ_DATA0)];
            case 5:
                _a.sent();
                Logger.success('Firmware length request recieved');
                fwLengthPacketBuffer = Buffer.alloc(5);
                fwLengthPacketBuffer[0] = BL_PACKET_FW_LENGTH_RES_DATA0;
                fwLengthPacketBuffer.writeUInt32LE(fwLength, 1);
                fwLengthPacket = new Packet(5, fwLengthPacketBuffer);
                writePacket(fwLengthPacket);
                Logger.info('Responding with firmware length');
                Logger.info('Waiting for a few seconds for main application to be erased...');
                return [4 /*yield*/, delay(1000)];
            case 6:
                _a.sent();
                Logger.info('Waiting for a few seconds for main application to be erased...');
                return [4 /*yield*/, delay(1000)];
            case 7:
                _a.sent();
                Logger.info('Waiting for a few seconds for main application to be erased...');
                return [4 /*yield*/, delay(1000)];
            case 8:
                _a.sent();
                bytesWritten = 0;
                _a.label = 9;
            case 9:
                if (!(bytesWritten < fwLength)) return [3 /*break*/, 11];
                return [4 /*yield*/, waitForSingleBytePacket(BL_PACKET_READY_FOR_DATA_DATA0)];
            case 10:
                _a.sent();
                dataBytes = fwImage.slice(bytesWritten, bytesWritten + PACKET_DATA_BYTES);
                dataLength = dataBytes.length;
                dataPacket = new Packet(dataLength - 1, dataBytes);
                writePacket(dataPacket);
                bytesWritten += dataLength;
                Logger.info("Wrote ".concat(dataLength, " bytes (").concat(bytesWritten, "/").concat(fwLength, ")"));
                return [3 /*break*/, 9];
            case 11: return [4 /*yield*/, waitForSingleBytePacket(BL_PACKET_UPDATE_SUCCESSFUL_DATA0)];
            case 12:
                _a.sent();
                Logger.success("Firmware update complete!");
                return [2 /*return*/];
        }
    });
}); };
main()
    .finally(function () { return uart.close(); });
