#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "Config.h"
#include "ModbusMotor.h"

class MotorController {
public:
    MotorController(ModbusMotor& left, ModbusMotor& right);
    void init();
    void setVelocity(float left, float right);
    void stop();
    void printTelemetry();
    
private:
    ModbusMotor& leftMotor;
    ModbusMotor& rightMotor;
    
    float targetLeft = 0.0f;
    float targetRight = 0.0f;
    uint32_t lastCommandMs = 0;

    char inputLine[96];
    uint8_t inputLen = 0;

    enum CommandMode {
    CMD_NONE = 0,
    CMD_RPM,
    CMD_CPS,
    CMD_CURRENT
    };

    CommandMode activeMode = CMD_NONE;

private:
    void zeroOffsets();
};