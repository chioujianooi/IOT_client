#include "modbus_client.h"
#include <cstring>
#include <iostream>

// ---------------------------------------------------------------------------
// Big-endian helpers
// ---------------------------------------------------------------------------

void ModbusClient::pushUint16BE(std::vector<uint8_t>& buf, uint16_t v) {
    buf.push_back(static_cast<uint8_t>(v >> 8));
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
}

uint16_t ModbusClient::readUint16BE(const uint8_t* buf) {
    return static_cast<uint16_t>((buf[0] << 8) | buf[1]);
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------

ModbusClient::ModbusClient(const std::string& ip, int port, uint8_t unit_id)
    : ip_(ip), port_(port), unitId_(unit_id) {}

ModbusClient::~ModbusClient() {
    disconnect();
}

// ---------------------------------------------------------------------------
// Connection lifecycle
// ---------------------------------------------------------------------------

bool ModbusClient::connect() {
    if (connected_)
        return true;

    if (!socket_.create()) {
        lastError_ = "Failed to create socket";
        return false;
    }

    socket_.customConnect(port_, ip_.c_str(), static_cast<int>(ip_.length()));
    connected_ = true;
    return true;
}

void ModbusClient::disconnect() {
    connected_ = false;
}

bool ModbusClient::isConnected() const {
    return connected_;
}

const std::string& ModbusClient::lastError() const {
    return lastError_;
}

// ---------------------------------------------------------------------------
// MBAP header
// ---------------------------------------------------------------------------

// frame must already be sized to at least 7 bytes (reserve then resize before calling).
// Overwrites bytes 0-6 with the MBAP header.
// The PDU length (bytes 4-5) is derived from frame.size() - 6 (everything after
// the first 6 MBAP bytes that precede the Length field itself is NOT correct;
// Length = number of bytes following byte 5, i.e. frame.size() - 6).
void ModbusClient::fillMbapHeader(std::vector<uint8_t>& frame) {
    // Transaction ID (bytes 0-1)
    frame[0] = static_cast<uint8_t>(transactionId_ >> 8);
    frame[1] = static_cast<uint8_t>(transactionId_ & 0xFF);
    // Protocol ID (bytes 2-3) — always 0x0000 for Modbus TCP
    frame[2] = 0x00;
    frame[3] = 0x00;
    // Length (bytes 4-5) — number of bytes that follow (Unit ID + PDU)
    uint16_t length = static_cast<uint16_t>(frame.size() - 6);
    frame[4] = static_cast<uint8_t>(length >> 8);
    frame[5] = static_cast<uint8_t>(length & 0xFF);
    // Unit ID (byte 6)
    frame[6] = unitId_;
}

// ---------------------------------------------------------------------------
// Low-level send/receive
// ---------------------------------------------------------------------------

bool ModbusClient::receiveExact(char* buf, int needed) {
    int total = 0;
    while (total < needed) {
        int n = socket_.receiveData(buf + total, needed - total);
        if (n <= 0) {
            lastError_ = "Connection closed or recv error";
            return false;
        }
        total += n;
    }
    return true;
}

bool ModbusClient::sendAndReceive(std::vector<uint8_t>& frame,
                                   std::vector<uint8_t>& response) {
    ++transactionId_;
    fillMbapHeader(frame);

    int sent = socket_.sendData(reinterpret_cast<const char*>(frame.data()),
                                static_cast<int>(frame.size()));
    if (sent <= 0) {
        lastError_ = "Failed to send request";
        return false;
    }

    // Phase 1: read the 7-byte MBAP header
    uint8_t mbap[7];
    if (!receiveExact(reinterpret_cast<char*>(mbap), 7))
        return false;

    // Extract PDU length from MBAP header (bytes 4-5)
    uint16_t length = readUint16BE(mbap + 4);
    if (length < 2) {
        lastError_ = "MBAP length field too small";
        return false;
    }

    // Phase 2: read the PDU (length includes Unit ID byte already read as byte 6)
    int pduSize = static_cast<int>(length) - 1; // subtract Unit ID
    std::vector<uint8_t> pdu(pduSize);
    if (!receiveExact(reinterpret_cast<char*>(pdu.data()), pduSize))
        return false;

    // Assemble: MBAP header (7B) + PDU
    response.resize(7 + pduSize);
    std::memcpy(response.data(), mbap, 7);
    std::memcpy(response.data() + 7, pdu.data(), pduSize);

    return true;
}

// ---------------------------------------------------------------------------
// Response validation
// ---------------------------------------------------------------------------

bool ModbusClient::validateResponse(const std::vector<uint8_t>& response,
                                     uint8_t expectedFc) {
    if (response.size() < 8) {
        lastError_ = "Response too short";
        return false;
    }

    // Check Protocol ID (bytes 2-3)
    if (response[2] != 0x00 || response[3] != 0x00) {
        lastError_ = "Invalid protocol ID in response";
        return false;
    }

    // Check Transaction ID (bytes 0-1)
    uint16_t rxTid = readUint16BE(response.data());
    if (rxTid != transactionId_) {
        lastError_ = "Transaction ID mismatch";
        return false;
    }

    // Check Unit ID (byte 6)
    if (response[6] != unitId_) {
        lastError_ = "Unit ID mismatch";
        return false;
    }

    // PDU starts at byte 7; byte 7 is the function code
    uint8_t fc = response[7];

    // High bit set → Modbus exception response
    if (fc & 0x80) {
        uint8_t exCode = (response.size() > 8) ? response[8] : 0;
        lastError_ = "Modbus exception, code: " + std::to_string(exCode);
        return false;
    }

    if (fc != expectedFc) {
        lastError_ = "Unexpected function code in response";
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// FC03 / FC04 — Read Registers
// ---------------------------------------------------------------------------

bool ModbusClient::readRegisters(uint8_t fc,
                                  uint16_t startAddr,
                                  uint16_t count,
                                  std::vector<uint16_t>& out) {
    if (!connected_) {
        lastError_ = "Not connected";
        return false;
    }

    // Build ADU: 7 MBAP + 1 FC + 2 addr + 2 count = 12 bytes
    std::vector<uint8_t> frame(12);
    frame[6] = unitId_;   // placeholder; fillMbapHeader will set bytes 0-6
    frame[7] = fc;
    frame[8]  = static_cast<uint8_t>(startAddr >> 8);
    frame[9]  = static_cast<uint8_t>(startAddr & 0xFF);
    frame[10] = static_cast<uint8_t>(count >> 8);
    frame[11] = static_cast<uint8_t>(count & 0xFF);

    std::vector<uint8_t> response;
    if (!sendAndReceive(frame, response))
        return false;
    if (!validateResponse(response, fc))
        return false;

    // Response PDU layout: [FC][ByteCount][Reg0_Hi][Reg0_Lo]...[RegN_Hi][RegN_Lo]
    // Byte 8 = ByteCount, data starts at byte 9
    if (response.size() < static_cast<size_t>(9 + count * 2)) {
        lastError_ = "Response data too short";
        return false;
    }

    out.resize(count);
    for (uint16_t i = 0; i < count; ++i)
        out[i] = readUint16BE(response.data() + 9 + i * 2);

    return true;
}

bool ModbusClient::readHoldingRegisters(uint16_t startAddr,
                                         uint16_t count,
                                         std::vector<uint16_t>& registers) {
    return readRegisters(0x03, startAddr, count, registers);
}

bool ModbusClient::readInputRegisters(uint16_t startAddr,
                                       uint16_t count,
                                       std::vector<uint16_t>& registers) {
    return readRegisters(0x04, startAddr, count, registers);
}

// ---------------------------------------------------------------------------
// FC06 — Write Single Register
// ---------------------------------------------------------------------------

bool ModbusClient::writeSingleRegister(uint16_t regAddr, uint16_t value) {
    if (!connected_) {
        lastError_ = "Not connected";
        return false;
    }

    // Build ADU: 7 MBAP + 1 FC + 2 addr + 2 value = 12 bytes
    std::vector<uint8_t> frame(12);
    frame[7] = 0x06;
    frame[8]  = static_cast<uint8_t>(regAddr >> 8);
    frame[9]  = static_cast<uint8_t>(regAddr & 0xFF);
    frame[10] = static_cast<uint8_t>(value >> 8);
    frame[11] = static_cast<uint8_t>(value & 0xFF);

    std::vector<uint8_t> response;
    if (!sendAndReceive(frame, response))
        return false;
    if (!validateResponse(response, 0x06))
        return false;

    return true;
}

// ---------------------------------------------------------------------------
// FC10 (0x10) — Write Multiple Registers
// ---------------------------------------------------------------------------

bool ModbusClient::writeMultipleRegisters(uint16_t startAddr,
                                           const std::vector<uint16_t>& values) {
    if (!connected_) {
        lastError_ = "Not connected";
        return false;
    }

    if (values.empty()) {
        lastError_ = "Values list must not be empty";
        return false;
    }

    if (values.size() > 123) {
        lastError_ = "Modbus FC10 limit: max 123 registers per request";
        return false;
    }

    uint16_t count     = static_cast<uint16_t>(values.size());
    uint8_t  byteCount = static_cast<uint8_t>(count * 2);

    // Build ADU: 7 MBAP + 1 FC + 2 addr + 2 count + 1 byteCount + count*2 data
    std::vector<uint8_t> frame(7 + 1 + 2 + 2 + 1 + byteCount);
    frame[7]  = 0x10;
    frame[8]  = static_cast<uint8_t>(startAddr >> 8);
    frame[9]  = static_cast<uint8_t>(startAddr & 0xFF);
    frame[10] = static_cast<uint8_t>(count >> 8);
    frame[11] = static_cast<uint8_t>(count & 0xFF);
    frame[12] = byteCount;

    for (uint16_t i = 0; i < count; ++i) {
        frame[13 + i * 2]     = static_cast<uint8_t>(values[i] >> 8);
        frame[13 + i * 2 + 1] = static_cast<uint8_t>(values[i] & 0xFF);
    }

    std::vector<uint8_t> response;
    if (!sendAndReceive(frame, response))
        return false;
    if (!validateResponse(response, 0x10))
        return false;

    return true;
}
