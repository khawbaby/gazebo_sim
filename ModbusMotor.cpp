#include "ModbusMotor.h"

ModbusMotor::ModbusMotor(HardwareSerial* _port, uint8_t _id, bool _reversed, bool _enabled, 
        uint16_t _mode, int32_t _zero, int32_t _lastPos, bool _hasLastPos, int TX_PIN, int RX_PIN) {
    port = _port;
    id = _id;
    reversed = _reversed;
    enabled = _enabled;
    mode = _mode;
    zero = _zero;
    lastPos = _lastPos;
    hasLastPos = _hasLastPos;
    TX = TX_PIN;
    RX = RX_PIN;
    
    
}


// --- HELPER FUNCTIONS ---
float ModbusMotor::clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float ModbusMotor::cpsToRpm(float cps) {
  return (cps / COUNTS_PER_REV) * 60.0f;
}

int32_t ModbusMotor::rpmToDriveUnits(float rpm) {
  return (int32_t)lround((rpm / 60.0f) * 1000.0f);
}

uint16_t ModbusMotor::crc16modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void ModbusMotor::rs485Tx(bool enable) {
  digitalWrite(RS485_DE_PIN, enable ? HIGH : LOW);
}

void ModbusMotor::clearRx() {
  while (port->available()) {
    port->read();
  }
}

bool ModbusMotor::crcOk(const uint8_t* buf, size_t len) {
  if (len < 4) return false;
  uint16_t calc = crc16modbus(buf, len - 2);
  return buf[len - 2] == (uint8_t)(calc & 0xFF) &&
         buf[len - 1] == (uint8_t)((calc >> 8) & 0xFF);
}

size_t ModbusMotor::exchangePacket(const uint8_t* tx, size_t txLen, uint8_t* rx, size_t rxMax, uint32_t timeoutMs) {
  clearRx();

  rs485Tx(true);
  delayMicroseconds(100);

  port->write(tx, txLen);
  port->flush();

  delayMicroseconds(100);
  rs485Tx(false);

  size_t n = 0;
  uint32_t start = millis();
  uint32_t lastByte = 0;
  bool seenData = false;

  while ((millis() - start) < timeoutMs) {
    while (port->available()) {
      int c = port->read();
      if (c >= 0 && n < rxMax) {
        rx[n++] = (uint8_t)c;
      }
      seenData = true;
      lastByte = millis();
    }

    if (seenData && (millis() - lastByte) > 10) {
      break;
    }
  }

  return n;
}

bool ModbusMotor::writeReg(uint16_t reg, uint16_t value) {
  uint8_t req[8];
  req[0] = id;
  req[1] = 0x06;
  req[2] = (reg >> 8) & 0xFF;
  req[3] = reg & 0xFF;
  req[4] = (value >> 8) & 0xFF;
  req[5] = value & 0xFF;

  uint16_t crc = crc16modbus(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  uint8_t resp[32];
  size_t n = exchangePacket(req, sizeof(req), resp, sizeof(resp));

  return n == 8 && crcOk(resp, n) && resp[0] == id && resp[1] == 0x06;
}

bool ModbusMotor::writeRegs(uint16_t startReg, const uint16_t* values, uint16_t count) {
  const uint8_t byteCount = count * 2;
  const size_t reqLen = 9 + byteCount;
  uint8_t req[64];

  req[0] = id;
  req[1] = 0x10;
  req[2] = (startReg >> 8) & 0xFF;
  req[3] = startReg & 0xFF;
  req[4] = (count >> 8) & 0xFF;
  req[5] = count & 0xFF;
  req[6] = byteCount;

  for (uint16_t i = 0; i < count; ++i) {
    req[7 + i * 2] = (values[i] >> 8) & 0xFF;
    req[8 + i * 2] = values[i] & 0xFF;
  }

  uint16_t crc = crc16modbus(req, 7 + byteCount);
  req[7 + byteCount] = crc & 0xFF;
  req[8 + byteCount] = (crc >> 8) & 0xFF;

  uint8_t resp[32];
  size_t n = exchangePacket(req, reqLen, resp, sizeof(resp));
  
  return n == 8 && crcOk(resp, n) && resp[0] == id && resp[1] == 0x10;
}

bool ModbusMotor::readRegs(uint16_t startReg, uint16_t count, uint16_t* out) {
  uint8_t req[8];
  req[0] = id;
  req[1] = 0x03;
  req[2] = (startReg >> 8) & 0xFF;
  req[3] = startReg & 0xFF;
  req[4] = (count >> 8) & 0xFF;
  req[5] = count & 0xFF;

  uint16_t crc = crc16modbus(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  uint8_t resp[64];
  size_t n = exchangePacket(req, sizeof(req), resp, sizeof(resp));

  const size_t expected = 5 + count * 2;
  if (n != expected) return false;
  if (!crcOk(resp, n)) return false;
  if (resp[0] != id || resp[1] != 0x03 || resp[2] != count * 2) return false;

  for (uint16_t i = 0; i < count; ++i) {
    out[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2];
  }
  return true;
}

bool ModbusMotor::readU16Reg(uint16_t reg, uint16_t& value) {
  return readRegs(reg, 1, &value);
}

bool ModbusMotor::readI16Reg(uint16_t reg, int16_t& value) {
  uint16_t raw = 0;
  if (!readRegs(reg, 1, &raw)) return false;
  value = (int16_t)raw;
  return true;
}

bool ModbusMotor::readI32Reg(uint16_t reg, int32_t& value) {
  uint16_t raw[2];
  if (!readRegs(reg, 2, raw)) return false;
  uint32_t packed = ((uint32_t)raw[0] << 16) | raw[1];
  value = (int32_t)packed;
  return true;
}

// --- basic control ---
bool ModbusMotor::clearFaults() {
    return writeReg(REG_CONTROL_WORD, CTRL_CLEAR_FAULT);
}

bool ModbusMotor::setMode(uint16_t m) {
    return writeReg(REG_WORK_MODE, m);
}

bool ModbusMotor::enableMotor() {
    if (!writeReg(REG_CONTROL_WORD, CTRL_ENABLE)) return false;
    enabled = true;
    return true;
}

bool ModbusMotor::disableMotor() {
    bool ok = writeReg(REG_CONTROL_WORD, CTRL_READY);
    enabled = false;
    mode = 0;
    return ok;
}


// --- mode management ---
bool ModbusMotor::ensureMode(uint16_t targetMode) {
    if (enabled && mode == targetMode) return true;

    if (!clearFaults()) return false;
    delay(50);

    if (!setMode(targetMode)) return false;
    delay(50);

    if (!enableMotor()) return false;
    delay(50);

    mode = targetMode;
    return true;
}


// --- speed control ---
bool ModbusMotor::setSpeedRpm(float rpm) {
    rpm = clampf(rpm, -MAX_RPM, MAX_RPM);
    if (reversed) rpm = -rpm;

    int32_t raw = rpmToDriveUnits(rpm);
    uint32_t u = (uint32_t)raw;

    uint16_t regs[2];
    regs[0] = (uint16_t)((u >> 16) & 0xFFFF);
    regs[1] = (uint16_t)(u & 0xFFFF);

    return writeRegs(REG_TARGET_SPEED, regs, 2);
}


// --- current control ---
bool ModbusMotor::setCurrentA(float amps) {
    amps = clampf(amps, -MAX_CURRENT_A, MAX_CURRENT_A);
    if (reversed) amps = -amps;

    int16_t raw = (int16_t)lround(amps * 1000.0f);
    return writeReg(REG_TARGET_CURRENT, (uint16_t)raw);
}

void ModbusMotor::stop() {
    if (mode == MODE_TORQUE) {
        ensureMode(MODE_TORQUE);
        setCurrentA(0.0f);
    } else {
        ensureMode(MODE_SPEED);
        setSpeedRpm(0.0f);
    }
}

bool ModbusMotor::readRawPosition(int32_t& pos) {
    if (readI32Reg(REG_ACTUAL_POS, pos)) {
        lastPos = pos;
        hasLastPos = true;
        return true;
    }

    if (hasLastPos) {
        pos = lastPos;
        return true;
    }

    pos = 0;
    return false;
}

int32_t ModbusMotor::readRelativeCounts() {
    int32_t raw = 0;
    readRawPosition(raw);

    int32_t rel = raw - zero;
    return reversed ? -rel : rel;
}

bool ModbusMotor::readBusVoltage(float& v) {
    uint16_t raw = 0;
    if (!readU16Reg(REG_BUS_VOLTAGE, raw)) return false;

    v = raw / 10.0f;
    return true;
}

bool ModbusMotor::readCurrent(float& i) {
    int16_t raw = 0;
    if (!readI16Reg(REG_ACTUAL_CURRENT, raw)) return false;

    i = raw / 1000.0f;
    return true;
}