#include <Arduino.h>
#include <math.h>
#include <string.h>

static const uint8_t LEFT_ID  = 1;
static const uint8_t RIGHT_ID = 1;

static const int LEFT_RX_PIN  = 1;
static const int LEFT_TX_PIN  = 2;
static const int RIGHT_RX_PIN = 5;
static const int RIGHT_TX_PIN = 4;
static const int RS485_DE_PIN = 16;

static const bool LEFT_REVERSED  = false;
static const bool RIGHT_REVERSED = true;

HardwareSerial leftBus(2);
HardwareSerial rightBus(1);

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

enum CommandMode {
  CMD_NONE = 0,
  CMD_RPM,
  CMD_CPS,
  CMD_CURRENT
};

struct Motor {
  HardwareSerial* port;
  uint8_t id;
  bool reversed;

  bool enabled;
  uint16_t mode;

  int32_t zero;
  int32_t lastPos;
  bool hasLastPos;
};

Motor leftMotor  = { &leftBus,  LEFT_ID,  LEFT_REVERSED,  false, 0, 0, 0, false };
Motor rightMotor = { &rightBus, RIGHT_ID, RIGHT_REVERSED, false, 0, 0, 0, false };

volatile CommandMode activeMode = CMD_NONE;
volatile float targetLeft = 0.0f;
volatile float targetRight = 0.0f;
volatile uint32_t lastCommandMs = 0;

static char inputLine[96];
static uint8_t inputLen = 0;

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static inline float cpsToRpm(float cps) {
  return (cps / COUNTS_PER_REV) * 60.0f;
}

static inline int32_t rpmToDriveUnits(float rpm) {
  return (int32_t)lround((rpm / 60.0f) * 1000.0f);
}

uint16_t crc16modbus(const uint8_t* data, size_t len) {
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

void rs485Tx(bool enable) {
  digitalWrite(RS485_DE_PIN, enable ? HIGH : LOW);
}

void clearRx(HardwareSerial& port) {
  while (port.available()) {
    port.read();
  }
}

bool crcOk(const uint8_t* buf, size_t len) {
  if (len < 4) return false;
  uint16_t calc = crc16modbus(buf, len - 2);
  return buf[len - 2] == (uint8_t)(calc & 0xFF) &&
         buf[len - 1] == (uint8_t)((calc >> 8) & 0xFF);
}

size_t exchangePacket(HardwareSerial& port,
                      const uint8_t* tx,
                      size_t txLen,
                      uint8_t* rx,
                      size_t rxMax,
                      uint32_t timeoutMs = 200) {
  clearRx(port);

  rs485Tx(true);
  delayMicroseconds(100);

  port.write(tx, txLen);
  port.flush();

  delayMicroseconds(100);
  rs485Tx(false);

  size_t n = 0;
  uint32_t start = millis();
  uint32_t lastByte = 0;
  bool seenData = false;

  while ((millis() - start) < timeoutMs) {
    while (port.available()) {
      int c = port.read();
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

bool writeReg(HardwareSerial& port, uint8_t id, uint16_t reg, uint16_t value) {
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
  size_t n = exchangePacket(port, req, sizeof(req), resp, sizeof(resp));

  return n == 8 && crcOk(resp, n) && resp[0] == id && resp[1] == 0x06;
}

bool writeRegs(HardwareSerial& port, uint8_t id, uint16_t startReg, const uint16_t* values, uint16_t count) {
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
  size_t n = exchangePacket(port, req, reqLen, resp, sizeof(resp));

  return n == 8 && crcOk(resp, n) && resp[0] == id && resp[1] == 0x10;
}

bool readRegs(HardwareSerial& port, uint8_t id, uint16_t startReg, uint16_t count, uint16_t* out) {
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
  size_t n = exchangePacket(port, req, sizeof(req), resp, sizeof(resp));

  const size_t expected = 5 + count * 2;
  if (n != expected) return false;
  if (!crcOk(resp, n)) return false;
  if (resp[0] != id || resp[1] != 0x03 || resp[2] != count * 2) return false;

  for (uint16_t i = 0; i < count; ++i) {
    out[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2];
  }
  return true;
}

bool readU16Reg(HardwareSerial& port, uint8_t id, uint16_t reg, uint16_t& value) {
  return readRegs(port, id, reg, 1, &value);
}

bool readI16Reg(HardwareSerial& port, uint8_t id, uint16_t reg, int16_t& value) {
  uint16_t raw = 0;
  if (!readRegs(port, id, reg, 1, &raw)) return false;
  value = (int16_t)raw;
  return true;
}

bool readI32Reg(HardwareSerial& port, uint8_t id, uint16_t reg, int32_t& value) {
  uint16_t raw[2];
  if (!readRegs(port, id, reg, 2, raw)) return false;
  uint32_t packed = ((uint32_t)raw[0] << 16) | raw[1];
  value = (int32_t)packed;
  return true;
}

bool clearFaults(Motor& m) {
  return writeReg(*m.port, m.id, REG_CONTROL_WORD, CTRL_CLEAR_FAULT);
}

bool setMode(Motor& m, uint16_t mode) {
  return writeReg(*m.port, m.id, REG_WORK_MODE, mode);
}

bool enableMotor(Motor& m) {
  if (!writeReg(*m.port, m.id, REG_CONTROL_WORD, CTRL_ENABLE)) return false;
  m.enabled = true;
  return true;
}

bool disableMotor(Motor& m) {
  bool ok = writeReg(*m.port, m.id, REG_CONTROL_WORD, CTRL_READY);
  m.enabled = false;
  m.mode = 0;
  return ok;
}

bool ensureMode(Motor& m, uint16_t mode) {
  if (m.enabled && m.mode == mode) return true;
  if (!clearFaults(m)) return false;
  delay(50);
  if (!setMode(m, mode)) return false;
  delay(50);
  if (!enableMotor(m)) return false;
  delay(50);
  m.mode = mode;
  return true;
}

bool setSpeedRpm(Motor& m, float rpm) {
  rpm = clampf(rpm, -MAX_RPM, MAX_RPM);
  if (m.reversed) rpm = -rpm;

  int32_t raw = rpmToDriveUnits(rpm);
  uint32_t u = (uint32_t)raw;

  uint16_t regs[2];
  regs[0] = (uint16_t)((u >> 16) & 0xFFFF);
  regs[1] = (uint16_t)(u & 0xFFFF);

  return writeRegs(*m.port, m.id, REG_TARGET_SPEED, regs, 2);
}

bool setCurrentA(Motor& m, float amps) {
  amps = clampf(amps, -MAX_CURRENT_A, MAX_CURRENT_A);
  if (m.reversed) amps = -amps;

  int16_t raw = (int16_t)lround(amps * 1000.0f);
  return writeReg(*m.port, m.id, REG_TARGET_CURRENT, (uint16_t)raw);
}

bool readRawPosition(Motor& m, int32_t& pos) {
  if (readI32Reg(*m.port, m.id, REG_ACTUAL_POS, pos)) {
    m.lastPos = pos;
    m.hasLastPos = true;
    return true;
  }

  if (m.hasLastPos) {
    pos = m.lastPos;
    return true;
  }

  pos = 0;
  return false;
}

int32_t readRelativeCounts(Motor& m) {
  int32_t raw = 0;
  readRawPosition(m, raw);
  int32_t rel = raw - m.zero;
  return m.reversed ? -rel : rel;
}

void stopMotor(Motor& m) {
  if (m.mode == MODE_TORQUE) {
    ensureMode(m, MODE_TORQUE);
    setCurrentA(m, 0.0f);
  } else {
    ensureMode(m, MODE_SPEED);
    setSpeedRpm(m, 0.0f);
  }
}

void printTelemetry() {
  uint16_t leftV = 0, rightV = 0;
  int16_t leftI = 0, rightI = 0;

  bool okLV = readU16Reg(*leftMotor.port, leftMotor.id, REG_BUS_VOLTAGE, leftV);
  bool okRV = readU16Reg(*rightMotor.port, rightMotor.id, REG_BUS_VOLTAGE, rightV);
  bool okLI = readI16Reg(*leftMotor.port, leftMotor.id, REG_ACTUAL_CURRENT, leftI);
  bool okRI = readI16Reg(*rightMotor.port, rightMotor.id, REG_ACTUAL_CURRENT, rightI);

  int32_t leftCount = readRelativeCounts(leftMotor);
  int32_t rightCount = readRelativeCounts(rightMotor);

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

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  e            -> print relative counts: L R");
  Serial.println("  m L R        -> speed in counts/sec");
  Serial.println("  o L R        -> speed in rpm");
  Serial.println("  d L R        -> current in amps");
  Serial.println("  t            -> telemetry");
  Serial.println("  x            -> zero count offsets");
  Serial.println("  z            -> stop both motors");
  Serial.println("  q            -> disable both motors");
  Serial.println("  h            -> help");
  Serial.println();
}

void zeroOffsets() {
  int32_t raw = 0;
  if (readRawPosition(leftMotor, raw)) leftMotor.zero = raw;
  if (readRawPosition(rightMotor, raw)) rightMotor.zero = raw;
}

void clearTargets() {
  targetLeft = 0.0f;
  targetRight = 0.0f;
  lastCommandMs = 0;
}

bool parseTwoFloats(const char* s, float& a, float& b) {
  return sscanf(s, "%f %f", &a, &b) == 2;
}

void handleLine() {
  if (inputLine[0] == '\0') return;

  if (strcmp(inputLine, "e") == 0) {
    Serial.print(readRelativeCounts(leftMotor));
    Serial.print(' ');
    Serial.println(readRelativeCounts(rightMotor));
    return;
  }

  if (strcmp(inputLine, "t") == 0) {
    printTelemetry();
    return;
  }

  if (strcmp(inputLine, "x") == 0) {
    zeroOffsets();
    Serial.println("OK");
    return;
  }

  if (strcmp(inputLine, "z") == 0) {
    stopMotor(leftMotor);
    stopMotor(rightMotor);
    clearTargets();
    Serial.println("OK");
    return;
  }

  if (strcmp(inputLine, "q") == 0) {
    stopMotor(leftMotor);
    stopMotor(rightMotor);
    delay(20);
    disableMotor(leftMotor);
    disableMotor(rightMotor);
    clearTargets();
    activeMode = CMD_NONE;
    Serial.println("OK");
    return;
  }

  if (strcmp(inputLine, "h") == 0) {
    printHelp();
    return;
  }

  float l = 0.0f;
  float r = 0.0f;

  if (inputLine[1] == ' ' && parseTwoFloats(inputLine + 2, l, r)) {
    if (inputLine[0] == 'o') {
      activeMode = CMD_RPM;
      targetLeft = l;
      targetRight = r;
      lastCommandMs = millis();
      Serial.println("OK");
      return;
    }

    if (inputLine[0] == 'm') {
      activeMode = CMD_CPS;
      targetLeft = l;
      targetRight = r;
      lastCommandMs = millis();
      Serial.println("OK");
      return;
    }

    if (inputLine[0] == 'd') {
      activeMode = CMD_CURRENT;
      targetLeft = l;
      targetRight = r;
      lastCommandMs = millis();
      Serial.println("OK");
      return;
    }
  }

  Serial.println("ERR");
}

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (inputLen > 0) {
        inputLine[inputLen] = '\0';
        handleLine();
        inputLen = 0;
      }
      continue;
    }

    if (inputLen < sizeof(inputLine) - 1) {
      inputLine[inputLen++] = c;
    } else {
      inputLen = 0;
    }
  }
}

void pushCommands() {
  switch (activeMode) {
    case CMD_RPM:
      if (ensureMode(leftMotor, MODE_SPEED)) setSpeedRpm(leftMotor, targetLeft);
      if (ensureMode(rightMotor, MODE_SPEED)) setSpeedRpm(rightMotor, targetRight);
      break;

    case CMD_CPS:
      if (ensureMode(leftMotor, MODE_SPEED)) setSpeedRpm(leftMotor, cpsToRpm(targetLeft));
      if (ensureMode(rightMotor, MODE_SPEED)) setSpeedRpm(rightMotor, cpsToRpm(targetRight));
      break;

    case CMD_CURRENT:
      if (ensureMode(leftMotor, MODE_TORQUE)) setCurrentA(leftMotor, targetLeft);
      if (ensureMode(rightMotor, MODE_TORQUE)) setCurrentA(rightMotor, targetRight);
      break;

    case CMD_NONE:
    default:
      stopMotor(leftMotor);
      stopMotor(rightMotor);
      break;
  }
}

void setup() {
  pinMode(RS485_DE_PIN, OUTPUT);
  rs485Tx(false);

  Serial.begin(USB_BAUD);
  delay(1000);

  Serial.println();
  Serial.println("Dual GMD6010-8 RS485 controller starting...");

  leftBus.begin(MOTOR_BAUD, SERIAL_8N1, LEFT_RX_PIN, LEFT_TX_PIN);
  rightBus.begin(MOTOR_BAUD, SERIAL_8N1, RIGHT_RX_PIN, RIGHT_TX_PIN);

  delay(300);

  zeroOffsets();

  uint16_t v = 0;

  Serial.print("Left bus voltage: ");
  if (readU16Reg(*leftMotor.port, leftMotor.id, REG_BUS_VOLTAGE, v)) Serial.println(v / 10.0f, 1);
  else Serial.println("nan");

  Serial.print("Right bus voltage: ");
  if (readU16Reg(*rightMotor.port, rightMotor.id, REG_BUS_VOLTAGE, v)) Serial.println(v / 10.0f, 1);
  else Serial.println("nan");

  printHelp();
}

void loop() {
  static uint32_t lastStreamMs = 0;

  pollSerial();

  uint32_t now = millis();
  if (now - lastStreamMs >= STREAM_PERIOD_MS) {
    lastStreamMs = now;

    bool fresh = (lastCommandMs != 0) && ((now - lastCommandMs) <= COMMAND_HOLD_MS);
    if (fresh) {
      pushCommands();
    } else {
      stopMotor(leftMotor);
      stopMotor(rightMotor);
    }
  }

  delay(1);
}