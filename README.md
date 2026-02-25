# 7Semi OPT4048 Arduino Library

Arduino library for the **TI OPT4048** color sensor (I2C).

This library reads the OPT4048 **X/Y/Z/W** channels, converts to **8-bit sRGB** (R,G,B),
and provides a simple **lux estimate** (from Y channel). It also includes a one-shot
**white-balance capture** helper to reduce color cast.

## Features

- I2C support 
- Read raw frame: X/Y/Z/W channels (exponent + mantissa decoded to float)
- Convert XYZ → sRGB (0..255)
- White balance capture (optional)
- Dark offset subtraction (optional)
- Simple lux estimate 

## Wiring (I2C)

| OPT4048 | MCU |
|---|---|
| VDD | 3.3V |
| GND | GND |
| SDA | SDA |
| SCL | SCL |

> Use **3.3V** logic/power unless your module explicitly supports 5V.


## Library Reference

### `begin(Wire, address)`
Initializes the OPT4048 over I2C and checks if the device responds.  
Returns `true` if the sensor is detected.

### `readConfig(uint16_t &cfg)`
Reads the current CONFIG register value.  
Returns `true` if the read was successful.

### `setConfigFull(mode, convTime, range, latch, intPolHigh, faultCount)`
Writes full configuration settings to the CONFIG register.  
Returns `true` if the write was successful.

### `setMode(mode)`
Sets the operating mode (shutdown, one-shot, continuous).  
Returns `true` if configuration was written successfully.

### `setConvTime(convTime)`
Sets the conversion time (integration time).  
Returns `true` if configuration was written successfully.

### `setRange(range)`
Sets the measurement range or enables auto-range.  
Returns `true` if configuration was written successfully.

### `setThresholds(low, high)`
Sets low and high interrupt threshold values.  
Returns `true` if both registers were written successfully.

### `isDataReady(timeoutMs)`
Waits until new data is ready or timeout occurs.  
Returns `true` when a fresh measurement is available.

### `Read(&data)`
Reads X/Y/Z/W channels, converts to 8-bit RGB, and provides a lux estimate.  
Returns `true` if the measurement was successful.

### `WhiteBalance()`
Captures current lighting as white reference for color correction.  
Returns `true` if white balance data was stored.

### `SetDarkOffset(x, y, z, w)`
Sets manual dark offset values for channel correction.  
Applies offset subtraction to future readings.

## Quick Start

```cpp
#include "7Semi_OPT4048.h"

OPT4048_7Semi opt;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!opt.begin(Wire, 0x44)) {
    Serial.println("OPT4048 not found");
    while (1) delay(500);
  }

  opt.setConfigFull(
    OPT4048_7Semi::CONTINUOUS,
    OPT4048_7Semi::CT_100MS,
    OPT4048_7Semi::AUTO
  );

  // Optional: capture WB once while pointing at a white surface
  // opt.WhiteBalance();
}

void loop() {
  OPT4048_7Semi::Data d;

  if (opt.isDataReady(250) && opt.Read(&d)) {
    Serial.print("RGB=");
    Serial.print(d.R8); Serial.print(",");
    Serial.print(d.G8); Serial.print(",");
    Serial.print(d.B8);

    Serial.print("  Lux~");
    Serial.println(d.lux, 2);
  }

  delay(50);
}
