#pragma once

#include "socket.h"
#include <cstdint>
#include <string>
#include <vector>

inline constexpr int         MODBUS_DEFAULT_PORT = 502;
inline constexpr const char* MODBUS_DEFAULT_IP   = "192.168.1.100";

class ModbusClient {
public:
    explicit ModbusClient(const std::string& ip      = MODBUS_DEFAULT_IP,
                          int                port    = MODBUS_DEFAULT_PORT,
                          uint8_t            unit_id = 0xFF);
    ~ModbusClient();

    ModbusClient(const ModbusClient&)            = delete;
    ModbusClient& operator=(const ModbusClient&) = delete;

    [[nodiscard]] bool connect();
    void               disconnect();
    [[nodiscard]] bool isConnected() const;

    // FC03 — Read Holding Registers
    [[nodiscard]] bool readHoldingRegisters(uint16_t startAddr,
                                            uint16_t count,
                                            std::vector<uint16_t>& registers);

    // FC04 — Read Input Registers
    [[nodiscard]] bool readInputRegisters(uint16_t startAddr,
                                          uint16_t count,
                                          std::vector<uint16_t>& registers);

    // FC06 — Write Single Register
    [[nodiscard]] bool writeSingleRegister(uint16_t regAddr, uint16_t value);

    // FC10 (0x10) — Write Multiple Registers (max 123 registers)
    [[nodiscard]] bool writeMultipleRegisters(uint16_t startAddr,
                                              const std::vector<uint16_t>& values);

    [[nodiscard]] const std::string& lastError() const;

private:
    // Shared implementation for FC03 and FC04 (identical frame layout)
    [[nodiscard]] bool readRegisters(uint8_t fc,
                                     uint16_t startAddr,
                                     uint16_t count,
                                     std::vector<uint16_t>& out);

    // Send a fully-built request frame; receive and return the complete response ADU
    [[nodiscard]] bool sendAndReceive(std::vector<uint8_t>& frame,
                                      std::vector<uint8_t>& response);

    // Validate MBAP fields and check for Modbus exception responses
    [[nodiscard]] bool validateResponse(const std::vector<uint8_t>& response,
                                        uint8_t expectedFc);

    // Read exactly 'needed' bytes from the socket into buf (handles TCP fragmentation)
    [[nodiscard]] bool receiveExact(char* buf, int needed);

    // Write bytes 0-6 (MBAP header) into an already-sized frame.
    // frame must be pre-sized to at least 7 bytes before calling.
    void fillMbapHeader(std::vector<uint8_t>& frame);

    static void     pushUint16BE(std::vector<uint8_t>& buf, uint16_t v);
    static uint16_t readUint16BE(const uint8_t* buf);

    std::string  ip_;
    int          port_;
    uint8_t      unitId_;
    bool         connected_     = false;
    uint16_t     transactionId_ = 0;
    std::string  lastError_;
    ClientSocket socket_;
};
