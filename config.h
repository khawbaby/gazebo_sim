#pragma once
#include <math.h>

static const uint8_t LEFT_ID  = 1;
static const uint8_t RIGHT_ID = 1;

static const int LEFT_RX_PIN  = 41;
static const int LEFT_TX_PIN  = 40;
static const int RIGHT_RX_PIN = 5;
static const int RIGHT_TX_PIN = 4;
static const int RS485_DE_PIN = 16;

static const bool LEFT_REVERSED  = false;
static const bool RIGHT_REVERSED = true;

static const uint16_t REG_BUS_VOLTAGE    = 31;
static const uint16_t REG_ACTUAL_CURRENT = 40;
static const uint16_t REG_ACTUAL_SPEED   = 41;
static const uint16_t REG_ACTUAL_POS     = 43;
static const uint16_t REG_WORK_MODE      = 60;
static const uint16_t REG_CONTROL_WORD   = 62;
static const uint16_t REG_TARGET_CURRENT = 64;
static const uint16_t REG_TARGET_SPEED   = 66;

static const uint16_t MODE_SPEED  = 2;
static const uint16_t MODE_TORQUE = 4;

static const uint16_t CTRL_READY       = 0x0006;
static const uint16_t CTRL_ENABLE      = 0x000F;
static const uint16_t CTRL_CLEAR_FAULT = 0x0086;

static const uint32_t USB_BAUD    = 115200;
static const uint32_t MOTOR_BAUD  = 38400;
static const float COUNTS_PER_REV = 131072.0f;

static const uint32_t STREAM_HZ        = 50;
static const uint32_t STREAM_PERIOD_MS = 1000 / STREAM_HZ;
static const uint32_t COMMAND_HOLD_MS  = 500;

static const float MAX_RPM       = 300.0f;
static const float MAX_CURRENT_A = 8.0f;
