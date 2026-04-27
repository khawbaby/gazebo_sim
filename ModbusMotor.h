#pragma once
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "Config.h"

class ModbusMotor {
public:
    ModbusMotor(HardwareSerial* _port, uint8_t _id, bool _reversed, bool _enabled, 
        uint16_t _mode, int32_t _zero, int32_t _lastPos, bool _hasLastPos, int TX_PIN, int RX_PIN);
    
    int TX;
    int RX;
    HardwareSerial* port;
    uint8_t id;
    bool reversed;

    bool enabled;
    uint16_t mode;

    int32_t zero;
    int32_t lastPos;
    bool hasLastPos;
    
    bool clearFaults();
    bool setMode(uint16_t mode);

    bool enableMotor();
    bool disableMotor();
    bool ensureMode(uint16_t targetMode);

    // --- control ---
    bool setSpeedRpm(float rpm);
    bool setCurrentA(float amps);

    // --- sensing ---
    bool readRawPosition(int32_t& pos);
    int32_t readRelativeCounts();

    // --- control ---
    void stop();

    // --- telemetry ---
    bool readBusVoltage(float& v);
    bool readCurrent(float& i);

private:
    // HELPER FUNCTION
    float clampf(float v, float lo, float hi);
    float cpsToRpm(float cps);
    int32_t rpmToDriveUnits(float rpm);
    uint16_t crc16modbus(const uint8_t* data, size_t len);

    void clearRx();
    bool crcOk(const uint8_t* buf, size_t len);
    size_t exchangePacket(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxMax, uint32_t timeoutMs = 200);

public: 
    bool writeReg(uint16_t reg, uint16_t value);
    bool writeRegs(uint16_t startReg, const uint16_t* values, uint16_t count);
   
    bool readRegs(uint16_t startReg, uint16_t count, uint16_t* out);
    bool readU16Reg(uint16_t reg, uint16_t& value);
    bool readI16Reg(uint16_t reg, int16_t& value);
    bool readI32Reg(uint16_t reg, int32_t& value);

    void rs485Tx(bool enable);

};