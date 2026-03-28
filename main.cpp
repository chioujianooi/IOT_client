#include "modbus_client.h"
#include <iostream>
#include <vector>

// Change these to match your Modbus TCP device
static constexpr const char* TARGET_IP   = "192.168.1.100";
static constexpr int         TARGET_PORT = 502;
static constexpr uint8_t     UNIT_ID     = 0x01;

int main() {
    ModbusClient client(TARGET_IP, TARGET_PORT, UNIT_ID);

    if (!client.connect()) {
        std::cerr << "Connect failed: " << client.lastError() << "\n";
        return 1;
    }

    // FC03: Read 10 holding registers starting at address 0
    std::vector<uint16_t> holdingRegs;
    if (!client.readHoldingRegisters(0, 10, holdingRegs)) {
        std::cerr << "FC03 error: " << client.lastError() << "\n";
    } else {
        for (size_t i = 0; i < holdingRegs.size(); ++i)
            std::cout << "HR[" << i << "] = " << holdingRegs[i] << "\n";
    }

    // FC04: Read 5 input registers starting at address 100
    std::vector<uint16_t> inputRegs;
    if (!client.readInputRegisters(100, 5, inputRegs)) {
        std::cerr << "FC04 error: " << client.lastError() << "\n";
    } else {
        for (size_t i = 0; i < inputRegs.size(); ++i)
            std::cout << "IR[" << i << "] = " << inputRegs[i] << "\n";
    }

    // FC06: Write value 1234 to register 10
    if (!client.writeSingleRegister(10, 1234)) {
        std::cerr << "FC06 error: " << client.lastError() << "\n";
    } else {
        std::cout << "FC06: wrote 1234 to register 10\n";
    }

    // FC10: Write three registers starting at address 20
    if (!client.writeMultipleRegisters(20, {0x0001, 0x0002, 0x0003})) {
        std::cerr << "FC10 error: " << client.lastError() << "\n";
    } else {
        std::cout << "FC10: wrote 3 registers starting at address 20\n";
    }

    client.disconnect();
    return 0;
}
