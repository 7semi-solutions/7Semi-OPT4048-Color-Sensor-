#pragma once

#include <Arduino.h>
#include <Wire.h>

/**
 * 7Semi OPT4048 driver (Arduino)
 *
 * - Safe I2C + register helpers
 * - Shadow CONFIG register so partial setters do not break other fields
 * - Reads X/Y/Z/W channels and converts to RGB (sRGB 8-bit)
 * - Provides white-balance capture to reduce green cast
 */

class OPT4048_7Semi {
public:

  /**
   * Operating mode (CONFIG[5..4])
   */
  enum Mode : uint8_t {
    POWERDOWN    = 0,
    AUTO_ONESHOT = 1,
    ONESHOT      = 2,
    CONTINUOUS   = 3
  };

  /**
   * Conversion time (CONFIG[9..6])
   *
   * - Values map directly to datasheet field
   */
  enum ConvTime : uint8_t {
    CT_600US  = 0,
    CT_1MS    = 1,
    CT_1P8MS  = 2,
    CT_3P4MS  = 3,
    CT_6P5MS  = 4,
    CT_12P7MS = 5,
    CT_25MS   = 6,
    CT_50MS   = 7,
    CT_100MS  = 8,
    CT_200MS  = 9,
    CT_400MS  = 10,
    CT_800MS  = 11
  };

  /**
   * Range (CONFIG[13..10])
   *
   * - 0..6 fixed ranges
   * - AUTO uses datasheet auto-range code (12)
   */
  enum Range : uint8_t {
    R0   = 0,
    R1   = 1,
    R2   = 2,
    R3   = 3,
    R4   = 4,
    R5   = 5,
    R6   = 6,
    AUTO = 12
  };

  /**
   * One channel sample unpacked from OPT4048 registers
   *
   * - exponent: 4-bit exponent from MSB register
   * - mantissa20: 20-bit mantissa reconstructed from MSB/LSB
   * - counter4: rolling conversion counter
   * - crc4: CRC nibble from device
   */
  struct Channel {
    uint8_t  exponent;
    uint32_t mantissa20;
    uint8_t  counter4;
    uint8_t  crc4;
  };

  /**
   * One full frame
   *
   * - X/Y/Z are tristimulus channels, W is wideband
   * - ready/overload come from FLAGS register
   */
  struct Frame {
    Channel X;
    Channel Y;
    Channel Z;
    Channel W;
    bool ready;
    bool overload;
  };

  /**
   * Output data
   *
   * - X/Y/Z/W are decoded floating values
   * - R/G/B are linear normalized color values [0..1] (color-only)
   * - R8/G8/B8 are gamma-corrected sRGB bytes [0..255]
   * - lux is a basic estimate from Y (needs calibration for accuracy)
   */
  struct Data {
    float X;
    float Y;
    float Z;
    float W;
    float R;
    float G;
    float B;
    uint8_t R8;
    uint8_t G8;
    uint8_t B8;
    float lux;
    bool ready;
    bool overload;
  };

  OPT4048_7Semi();

  /**
   * begin
   *
   * - Stores Wire instance and address
   * - Starts I2C with optional custom SDA/SCL on ESP32
   * - Pings device, reads DEVICE_ID, reads CONFIG into shadow
   * - Does NOT overwrite CONFIG automatically
   */
  bool begin(TwoWire& i2cCom,
             uint8_t i2cAddress = 0x44,
             int sda = -1,
             int scl = -1,
             uint32_t clockSpeed = 400000);

  /**
   * readDeviceId
   *
   * - Reads device ID register
   */
  bool readDeviceId(uint16_t& id);

  /**
   * readConfig
   *
   * - Reads CONFIG register into outCfg
   * - Also updates internal shadow
   */
  bool readConfig(uint16_t& outCfg);

  /**
   * setConfigFull
   *
   * - Builds and writes full CONFIG word from provided fields
   * - Updates internal shadow on success
   */
  bool setConfigFull(Mode m, ConvTime ct, Range r,
                     bool latch = true, bool intPolHigh = false, uint8_t faultCount = 0);

  /**
   * Partial config setters (SAFE)
   *
   * - These functions modify only their bits and keep other bits unchanged
   * - They write CONFIG immediately (so change is applied only when you call them)
   */
  bool setMode(Mode m);
  bool setConvTime(ConvTime ct);
  bool setRange(Range r);
  bool setInterrupt(bool latch, bool intPolHigh, uint8_t faultCount);

  /**
   * isDataReady
   *
   * - Polls FLAGS until conversion ready or timeout
   */
  bool isDataReady(uint32_t timeoutMs = 100);

  /**
   * readFrame
   *
   * - Reads raw registers 0x00..0x07 (X/Y/Z/W) and FLAGS
   * - Returns unpacked Frame
   */
  bool readFrame(Frame& frame);

  /**
   * decodeChannel
   *
   * - Converts mantissa/exponent into a float using ldexpf()
   * - Prevents overflow that breaks RGB
   */
  static float decodeChannel(const Channel& c);

  /**
   * WhiteBalance
   *
   * - Captures white-balance gains from current X/Y/Z reading
   * - Use a WHITE paper under your main light
   */
  bool WhiteBalance();

  /**
   * clearWhiteBalance
   *
   * - Disables WB gains (back to 1.0)
   */
  void clearWhiteBalance();

  /**
   * Read
   *
   * - Reads frame, decodes X/Y/Z/W
   * - Applies dark offsets
   * - Applies white-balance gains (if ready)
   * - Converts XYZ chromaticity -> sRGB bytes
   */
  bool Read(Data* out);

  /**
   * SetDarkOffset
   *
   * - Sets dark offsets to subtract from decoded XYZ
   */
  void SetDarkOffset(float dx, float dy, float dz);

private:
  /**
   * Register map (minimal)
   */
  static constexpr uint8_t REG_CONFIG    = 0x0A;
  static constexpr uint8_t REG_FLAGS     = 0x0C;
  static constexpr uint8_t REG_DEVICE_ID = 0x11;

  /**
   * I2C helpers
   */
  bool readReg(uint8_t reg, uint16_t& val);
  bool writeReg(uint8_t reg, uint16_t val);
  bool readSeq(uint8_t startReg, uint16_t* buf, uint8_t nWords);

  /**
   * readStatus
   *
   * - Reads FLAGS and extracts ready/overload
   */
  bool readStatus(bool& convReady, bool& overload);

  /**
   * parseChan
   *
   * - Unpacks one channel from 2 words
   */
  static void parseChan(uint16_t msbexp, uint16_t lsbcrc, Channel& ch);

  /**
   * xyzToSrgbChromaticity
   *
   * - Uses X/Y/Z normalized by sum to get stable color
   * - Converts with standard D65 XYZ->sRGB matrix
   * - Clips negatives, normalizes max to 1.0
   */
  static void xyzToSrgbChromaticity(float X, float Y, float Z,
                                    float& Rlin, float& Glin, float& Blin,
                                    uint8_t& R8, uint8_t& G8, uint8_t& B8);

  TwoWire* i2c = nullptr;
  uint8_t address = 0x44;

  /**
   * Dark offsets
   */
  float dark[3] = {0.0f, 0.0f, 0.0f};

  /**
   * White balance gains
   */
  float white_balance_g[3] = {1.0f, 1.0f, 1.0f};
  bool white_balance_ready = false;

  /**
 * sRGB constants (D65, 2°)
 *
 * - Matrix converts XYZ (relative) -> linear sRGB
 * - Standard sRGB / IEC 61966-2-1
 */
static constexpr float kXYZ_to_sRGB[3][3] = {
  {  3.2406f, -1.5372f, -0.4986f },
  { -0.9689f,  1.8758f,  0.0415f },
  {  0.0557f, -0.2040f,  1.0570f }
};

/**
 * linear -> sRGB companding
 *
 * - Input is linear in [0..1]
 */
static inline float to_srgb(float L) {
  if (L <= 0.0031308f) return 12.92f * L;
  return 1.055f * powf(L, 1.0f / 2.4f) - 0.055f;
}
};
