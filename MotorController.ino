#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "Config.h"
#include "ModbusMotor.h"
#include "MotorController.h"

HardwareSerial leftBus(2);
HardwareSerial rightBus(1);

ModbusMotor leftMotor(&leftBus,  LEFT_ID,  LEFT_REVERSED,  false, 0, 0, 0, false, LEFT_TX_PIN, LEFT_RX_PIN);
ModbusMotor rightMotor(&rightBus, RIGHT_ID, RIGHT_REVERSED, false, 0, 0, 0, false, RIGHT_TX_PIN, RIGHT_RX_PIN);

MotorController controller(leftMotor, rightMotor);

void setup() {

    rightMotor.port->begin(MOTOR_BAUD, SERIAL_8N1, RIGHT_RX_PIN, RIGHT_TX_PIN);
    leftMotor.port->begin(MOTOR_BAUD, SERIAL_8N1, LEFT_RX_PIN, LEFT_TX_PIN);
//   Serial.begin(USB_BAUD);
//   delay(1000);

//   Serial.println();
//   Serial.println("Dual GMD6010-8 RS485 controller starting...");

//   leftBus.begin(MOTOR_BAUD, SERIAL_8N1, LEFT_RX_PIN, LEFT_TX_PIN);
//   rightBus.begin(MOTOR_BAUD, SERIAL_8N1, RIGHT_RX_PIN, RIGHT_TX_PIN);

//   delay(300);

//   zeroOffsets();

//   uint16_t v = 0;

//   Serial.print("Left bus voltage: ");
//   if (readU16Reg(*leftMotor.port, leftMotor.id, REG_BUS_VOLTAGE, v)) Serial.println(v / 10.0f, 1);
//   else Serial.println("nan");

//   Serial.print("Right bus voltage: ");
//   if (readU16Reg(*rightMotor.port, rightMotor.id, REG_BUS_VOLTAGE, v)) Serial.println(v / 10.0f, 1);
//   else Serial.println("nan");

//   printHelp();
}

void loop() {
    controller.setVelocity(1000, 1000);
    controller.printTelemetry();
    delay(2000);

    controller.stop();
    delay(1000);

    
}