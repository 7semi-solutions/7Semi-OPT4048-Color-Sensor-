#include "7Semi_OPT4048.h"
#include <math.h>



OPT4048_7Semi::OPT4048_7Semi() {}

bool OPT4048_7Semi::begin(TwoWire& i2cCom,
                          uint8_t i2cAddress,
                          int sda,
                          int scl,
                          uint32_t clockSpeed) {
  i2c = &i2cCom;
  address = i2cAddress;

#if defined(ARDUINO_ARCH_ESP32)
  if (sda >= 0 && scl >= 0) i2c->begin(sda, scl);
  else i2c->begin();
#else
  (void)sda;
  (void)scl;
  i2c->begin();
#endif

  i2c->setClock(clockSpeed);
  delay(10);

  /**
   * Ping
   */
  i2c->beginTransmission(address);
  if (i2c->endTransmission() != 0) return false;

  /**
   * Check ID
   */
  uint16_t id = 0;
  if (!readDeviceId(id)) return false;

  /**
   * Common check:
   * - Some modules return ID in lower bits
   */
  if ((id & 0x0FFF) == 0x0FFF) return false;

  /**
   * Load current CONFIG into shadow
   *
   * - This ensures your partial setters don't destroy unknown defaults
   */
  if(!setConfigFull(
    CONTINUOUS,
    CT_100MS,
    AUTO,
    true,
    false,
    0
  ))
  return false;


  return true;
}

bool OPT4048_7Semi::readDeviceId(uint16_t& id) {
  return readReg(REG_DEVICE_ID, id);
}

bool OPT4048_7Semi::readConfig(uint16_t& outCfg) {
  if (!readReg(REG_CONFIG, outCfg)) return false;
  return true;
}

bool OPT4048_7Semi::setConfigFull(Mode m, ConvTime ct, Range r,
                                  bool latch, bool intPolHigh, uint8_t faultCount) {
  /**
   * CONFIG (0x0A) packing per datasheet
   *
   * - bits13..10: RANGE
   * - bits9..6:  CONVERSION_TIME
   * - bits5..4:  OPERATING_MODE
   * - bit3:      LATCH
   * - bit2:      INT_POL
   * - bits1..0:  FAULT_COUNT
   */
  uint16_t cfg = 0;
  cfg |= (uint16_t(r & 0x0F) << 10);
  cfg |= (uint16_t(ct & 0x0F) << 6);
  cfg |= (uint16_t(m & 0x03) << 4);
  if (latch) cfg |= (1u << 3);
  if (intPolHigh) cfg |= (1u << 2);
  cfg |= (uint16_t(faultCount & 0x03) << 0);

  if (!writeReg(REG_CONFIG, cfg)) return false;
  return true;
}

bool OPT4048_7Semi::setMode(Mode m) {
    uint16_t v;
    if (!readConfig(v)) return false;

  /**
   * Update only MODE bits [5..4]
   */
  v &= ~(uint16_t(0x03) << 4);
  v |= (uint16_t(m & 0x03) << 4);

  if (!writeReg(REG_CONFIG, v)) return false;
  return true;
}

bool OPT4048_7Semi::setConvTime(ConvTime ct) {
    uint16_t v;
    if (!readConfig(v)) return false;

  /**
   * Update only CONVERSION_TIME bits [9..6]
   */
  v &= ~(uint16_t(0x0F) << 6);
  v |= (uint16_t(ct & 0x0F) << 6);

  if (!writeReg(REG_CONFIG, v)) return false;
  return true;
}

bool OPT4048_7Semi::setRange(Range r) {
    uint16_t v;
    if (!readConfig(v)) return false;

  /**
   * Update only RANGE bits [13..10]
   */
  v &= ~(uint16_t(0x0F) << 10);
  v |= (uint16_t(r & 0x0F) << 10);

  if (!writeReg(REG_CONFIG, v)) return false;
  return true;
}

bool OPT4048_7Semi::setInterrupt(bool latch, bool intPolHigh, uint8_t faultCount) {
    uint16_t v;
    if (!readConfig(v)) return false;


  /**
   * Update only:
   * - LATCH bit [3]
   * - INT_POL bit [2]
   * - FAULT_COUNT bits [1..0]
   */
  v &= ~(uint16_t(1u) << 3);
  v &= ~(uint16_t(1u) << 2);
  v &= ~(uint16_t(0x03) << 0);

  if (latch) v |= (uint16_t(1u) << 3);
  if (intPolHigh) v |= (uint16_t(1u) << 2);
  v |= (uint16_t(faultCount & 0x03) << 0);

  if (!writeReg(REG_CONFIG, v)) return false;
  return true;
}

bool OPT4048_7Semi::isDataReady(uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    bool ready = false;
    bool ov = false;
    if (!readStatus(ready, ov)) return false;
    if (ready) return true;
    delay(2);
  }
  return false;
}

bool OPT4048_7Semi::readFrame(Frame& frame) {
  uint16_t raw[8];
  if (!readSeq(0x00, raw, 8)) return false;

  parseChan(raw[0], raw[1], frame.X);
  parseChan(raw[2], raw[3], frame.Y);
  parseChan(raw[4], raw[5], frame.Z);
  parseChan(raw[6], raw[7], frame.W);

  return readStatus(frame.ready, frame.overload);
}

float OPT4048_7Semi::decodeChannel(const Channel& c) {
  /**
   * Safe decode
   *
   * - True code value is mantissa * 2^exponent
   * - Using ldexpf avoids integer overflow from (mantissa<<exponent)
   */
  return ldexpf((float)c.mantissa20, (int)c.exponent);
}

void OPT4048_7Semi::SetDarkOffset(float dx, float dy, float dz) {
  dark[0] = dx;
  dark[1] = dy;
  dark[2] = dz;
}

bool OPT4048_7Semi::WhiteBalance() {
  /**
   * White balance capture
   *
   * - Read current X/Y/Z
   * - Compute avg/channel gains (STM-style)
   * - Gains reduce green cast and stabilize hue
   */
  Frame f;
  if (!readFrame(f)) return false;

  float X = decodeChannel(f.X) - dark[0];
  float Y = decodeChannel(f.Y) - dark[1];
  float Z = decodeChannel(f.Z) - dark[2];

  if (X <= 0.0f) X = 1.0f;
  if (Y <= 0.0f) Y = 1.0f;
  if (Z <= 0.0f) Z = 1.0f;

  const float avg = (X + Y + Z) / 3.0f;

  white_balance_g[0] = avg / X;
  white_balance_g[1] = avg / Y;
  white_balance_g[2] = avg / Z;

  white_balance_ready = true;
  return true;
}

void OPT4048_7Semi::clearWhiteBalance() {
  white_balance_g[0] = 1.0f;
  white_balance_g[1] = 1.0f;
  white_balance_g[2] = 1.0f;
  white_balance_ready = false;
}

bool OPT4048_7Semi::Read(Data* out) {
  if (!out) return false;

  Frame f;
  if (!readFrame(f)) return false;

  float X = decodeChannel(f.X) - dark[0];
  float Y = decodeChannel(f.Y) - dark[1];
  float Z = decodeChannel(f.Z) - dark[2];
  float W = decodeChannel(f.W);

  if (X < 0.0f) X = 0.0f;
  if (Y < 0.0f) Y = 0.0f;
  if (Z < 0.0f) Z = 0.0f;

  /**
   * Apply WB gains
   */
  float Xw = X, Yw = Y, Zw = Z;
  if (white_balance_ready) {
    Xw *= white_balance_g[0];
    Yw *= white_balance_g[1];
    Zw *= white_balance_g[2];
  }

  float Rlin = 0.0f, Glin = 0.0f, Blin = 0.0f;
  uint8_t R8 = 0, G8 = 0, B8 = 0;

  xyzToSrgbChromaticity(Xw, Yw, Zw, Rlin, Glin, Blin, R8, G8, B8);

  /**
   * Lux estimate
   *
   * - Simple scaling from Y channel (starting point)
   * - Calibrate factor with lux meter for accuracy
   */
  const float kLuxPerYCode = 2.15e-3f;
  float lux = (Y > 0.0f) ? (Y * kLuxPerYCode) : 0.0f;

  out->X = X;
  out->Y = Y;
  out->Z = Z;
  out->W = W;
  out->R = Rlin;
  out->G = Glin;
  out->B = Blin;
  out->R8 = R8;
  out->G8 = G8;
  out->B8 = B8;
  out->lux = lux;
  out->ready = f.ready;
  out->overload = f.overload;

  return true;
}

bool OPT4048_7Semi::readStatus(bool& convReady, bool& overload) {
  uint16_t f = 0;
  if (!readReg(REG_FLAGS, f)) return false;

  /**
   * Datasheet flags:
   * - bit2: conversion ready
   * - bit3: overload
   */
  convReady = (f & (1u << 2)) != 0;
  overload  = (f & (1u << 3)) != 0;
  return true;
}

void OPT4048_7Semi::parseChan(uint16_t msbexp, uint16_t lsbcrc, Channel& ch) {
  ch.exponent   = (msbexp >> 12) & 0x0F;
  ch.mantissa20 = ((uint32_t)(msbexp & 0x0FFF) << 8) | ((lsbcrc >> 8) & 0xFF);
  ch.counter4   = (lsbcrc >> 4) & 0x0F;
  ch.crc4       = (lsbcrc >> 0) & 0x0F;
}

bool OPT4048_7Semi::readReg(uint8_t reg, uint16_t& val) {
  i2c->beginTransmission(address);
  i2c->write(reg);
  if (i2c->endTransmission(false) != 0) return false;

  if (i2c->requestFrom((int)address, 2) != 2) return false;
  const uint8_t msb = i2c->read();
  const uint8_t lsb = i2c->read();
  val = (uint16_t(msb) << 8) | lsb;
  return true;
}

bool OPT4048_7Semi::writeReg(uint8_t reg, uint16_t val) {
  i2c->beginTransmission(address);
  i2c->write(reg);
  i2c->write((uint8_t)(val >> 8));
  i2c->write((uint8_t)(val >> 0));
  return i2c->endTransmission() == 0;
}

bool OPT4048_7Semi::readSeq(uint8_t startReg, uint16_t* buf, uint8_t nWords) {
  i2c->beginTransmission(address);
  i2c->write(startReg);
  if (i2c->endTransmission(false) != 0) return false;

  const int need = int(nWords) * 2;
  if (i2c->requestFrom((int)address, need) != need) return false;

  for (uint8_t i = 0; i < nWords; i++) {
    const uint8_t msb = i2c->read();
    const uint8_t lsb = i2c->read();
    buf[i] = (uint16_t(msb) << 8) | lsb;
  }
  return true;
}

void OPT4048_7Semi::xyzToSrgbChromaticity(float X, float Y, float Z,
                                         float& Rlin, float& Glin, float& Blin,
                                         uint8_t& R8, uint8_t& G8, uint8_t& B8) {
  /**
   * XYZ -> sRGB using chromaticity
   *
   * - Normalize by sum so hue is stable vs brightness
   * - Convert using standard XYZ->sRGB matrix
   * - Clip negatives (out of gamut)
   * - Normalize max to 1.0 for display-friendly RGB
   */
  auto clamp01 = [](float v) {
    return (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v);
  };

  const float sum = X + Y + Z;
  if (sum <= 0.0f) {
    Rlin = Glin = Blin = 0.0f;
    R8 = G8 = B8 = 0;
    return;
  }

  const float Xn = X / sum;
  const float Yn = Y / sum;
  const float Zn = Z / sum;

  float r = kXYZ_to_sRGB[0][0] * Xn + kXYZ_to_sRGB[0][1] * Yn + kXYZ_to_sRGB[0][2] * Zn;
  float g = kXYZ_to_sRGB[1][0] * Xn + kXYZ_to_sRGB[1][1] * Yn + kXYZ_to_sRGB[1][2] * Zn;
  float b = kXYZ_to_sRGB[2][0] * Xn + kXYZ_to_sRGB[2][1] * Yn + kXYZ_to_sRGB[2][2] * Zn;

  if (r < 0.0f) r = 0.0f;
  if (g < 0.0f) g = 0.0f;
  if (b < 0.0f) b = 0.0f;

  const float maxv = fmaxf(r, fmaxf(g, b));
  if (maxv > 0.0f) {
    r /= maxv;
    g /= maxv;
    b /= maxv;
  }

  Rlin = clamp01(r);
  Glin = clamp01(g);
  Blin = clamp01(b);

  const float Rs = to_srgb(Rlin);
  const float Gs = to_srgb(Glin);
  const float Bs = to_srgb(Blin);

  R8 = (uint8_t)(clamp01(Rs) * 255.0f + 0.5f);
  G8 = (uint8_t)(clamp01(Gs) * 255.0f + 0.5f);
  B8 = (uint8_t)(clamp01(Bs) * 255.0f + 0.5f);
}