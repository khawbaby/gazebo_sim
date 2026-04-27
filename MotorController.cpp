#include "MotorController.h"
#include "Config.h"

MotorController::MotorController(ModbusMotor& left, ModbusMotor& right) : leftMotor(left), rightMotor(right) {
    Serial.begin(USB_BAUD);
    delay(1000);

    Serial.println();
    Serial.println("Dual GMD6010-8 RS485 controller starting...");
    init();

    uint16_t v = 0;

    Serial.print("Left bus voltage: ");
    if (leftMotor.readU16Reg(REG_BUS_VOLTAGE, v)) Serial.println(v / 10.0f, 1);
    else Serial.println("nan");

    Serial.print("Right bus voltage: ");
    if (rightMotor.readU16Reg(REG_BUS_VOLTAGE, v)) Serial.println(v / 10.0f, 1);
    else Serial.println("nan");
}

void MotorController::init() {
    pinMode(RS485_DE_PIN, OUTPUT);
    zeroOffsets();
}

void MotorController::setVelocity(float left, float right) {
    if (leftMotor.ensureMode(MODE_SPEED)) leftMotor.setSpeedRpm(left);
    if (rightMotor.ensureMode(MODE_SPEED)) rightMotor.setSpeedRpm(right);
}


void MotorController::zeroOffsets() {
    int32_t raw = 0;
    if (leftMotor.readRawPosition(raw)) leftMotor.zero = raw;
    if (leftMotor.readRawPosition(raw)) rightMotor.zero = raw;
}

void MotorController::stop() {
  if (leftMotor.mode == MODE_TORQUE) {
    leftMotor.ensureMode(MODE_TORQUE);
    leftMotor.setCurrentA(0.0f);
    rightMotor.ensureMode(MODE_TORQUE);
    rightMotor.setCurrentA(0.0f);

  } else {
    leftMotor.ensureMode(MODE_SPEED);
    leftMotor.setSpeedRpm(0.0f);
    rightMotor.ensureMode(MODE_SPEED);
    rightMotor.setSpeedRpm(0.0f);
  }
}


void MotorController::printTelemetry() {
  uint16_t leftV = 0, rightV = 0;
  int16_t leftI = 0, rightI = 0;

  bool okLV = leftMotor.readU16Reg(REG_BUS_VOLTAGE, leftV);
  bool okRV = rightMotor.readU16Reg(REG_BUS_VOLTAGE, rightV);
  bool okLI = leftMotor.readI16Reg(REG_ACTUAL_CURRENT, leftI);
  bool okRI = rightMotor.readI16Reg(REG_ACTUAL_CURRENT, rightI);

  int32_t leftCount = leftMotor.readRelativeCounts();
  int32_t rightCount = rightMotor.readRelativeCounts();

  Serial.print("L V=");
  if (okLV) Serial.print(leftV / 10.0f, 1); else Serial.print("nan");
  Serial.print(" I=");
  if (okLI) Serial.print(leftI / 1000.0f, 3); else Serial.print("nan");
  Serial.print(" cnt=");
  Serial.print(leftCount);

  Serial.print(" | R V=");
  if (okRV) Serial.print(rightV / 10.0f, 1); else Serial.print("nan");
  Serial.print(" I=");
  if (okRI) Serial.print(rightI / 1000.0f, 3); else Serial.print("nan");
  Serial.print(" cnt=");
  Serial.print(rightCount);

  Serial.println();
}