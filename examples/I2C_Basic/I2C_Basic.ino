/**
 * 7Semi OPT4048 - Basic I2C Example
 *
 * Library Information:
 * - Library Name : 7Semi_OPT4048
 * - Sensor       : Texas Instruments OPT4048
 * - Interface    : I2C only
 * - Function     : Reads X/Y/Z/W channels and converts to 8-bit RGB + Lux
 *
 * I2C Connection:
 * -------------------------------------------------
 * OPT4048      ->   Microcontroller
 * -------------------------------------------------
 * VDD          ->   3.3V
 * GND          ->   GND
 * SDA          ->   SDA (I2C Data)
 * SCL          ->   SCL (I2C Clock)
 *
 * Notes:
 * - Default I2C address is 0x44
 * - Use 3.3V logic (recommended)
 *
 * What This Example Does:
 * - Initializes OPT4048
 * - Sets continuous measurement mode
 * - Reads RGB (0-255) and Lux
 * - Prints values to Serial Monitor
 */
#include <7Semi_OPT4048.h>
OPT4048_7Semi opt;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  /**
   * Initialize sensor
   * - Uses default Wire bus
   * - Uses default I2C address 0x44
   */
  if (!opt.begin(Wire, 0x44)) {
    Serial.println("OPT4048 not detected!");
    while (1) delay(500);
  }

  /**
   * Configure sensor
   * - Continuous measurement mode
   * - 100ms conversion time
   * - Auto range enabled
   */
  opt.setConfigFull(
    OPT4048_7Semi::CONTINUOUS,
    OPT4048_7Semi::CT_100MS,
    OPT4048_7Semi::AUTO);

  if (opt.WhiteBalance())
    Serial.println("White balance stored");
  else
    Serial.println("White balance failed");

  Serial.println("OPT4048 initialized successfully.");
}

void loop() {
  OPT4048_7Semi::Data data;

  /**
   * Wait for new data
   * - Timeout set to 250 ms
   */
  if (opt.isDataReady(250) && opt.Read(&data)) {
    Serial.print("RGB: ");
    Serial.print(data.R8);
    Serial.print(", ");
    Serial.print(data.G8);
    Serial.print(", ");
    Serial.print(data.B8);

    Serial.print("  |  Lux: ");
    Serial.print(data.lux, 2);

    Serial.print("  |  Overload: ");
    Serial.println(data.overload ? "YES" : "NO");
  }

  delay(100);
}