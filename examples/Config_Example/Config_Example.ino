/**
 * 7Semi OPT4048 - Configuration Example
 *
 * Library Information:
 * - Library Name : 7Semi_OPT4048
 * - Sensor       : Texas Instruments OPT4048
 * - Interface    : I2C only
 * - Function     : Demonstrates manual configuration settings
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
 * What This Example Does:
 * - Initializes OPT4048
 * - Sets custom mode, conversion time, and range
 * - Reads configuration register
 * - Prints configuration and RGB data
 */
#include <7Semi_OPT4048.h>

OPT4048_7Semi opt;

void setup()
{
  Serial.begin(115200);
  while (!Serial) {}

  /**
   * Initialize sensor at default I2C address 0x44
   */
  if (!opt.begin(Wire, 0x44))
  {
    Serial.println("OPT4048 not detected!");
    while (1) delay(500);
  }

  /**
   * Custom Configuration
   * - Mode        : Continuous measurement
   * - Conv Time   : 200ms integration
   * - Range       : AUTO
   * - Latch       : Enabled
   * - INT Polarity: Active High
   * - Fault Count : 1
   */
  if (!opt.setConfigFull(
        OPT4048_7Semi::CONTINUOUS,
        OPT4048_7Semi::CT_200MS,
        OPT4048_7Semi::AUTO,
        true,
        true,
        1))
  {
    Serial.println("Configuration write failed!");
    while (1) delay(500);
  }

  /**
   * Read back configuration register
   */
  uint16_t configReg;
  if (opt.readConfig(configReg))
  {
    Serial.print("CONFIG Register: 0x");
    Serial.println(configReg, HEX);
  }

  Serial.println("OPT4048 configured successfully.");
}

void loop()
{
  OPT4048_7Semi::Data data;

  if (opt.isDataReady(300) && opt.Read(&data))
  {
    Serial.print("RGB: ");
    Serial.print(data.R8);
    Serial.print(", ");
    Serial.print(data.G8);
    Serial.print(", ");
    Serial.print(data.B8);

    Serial.print("  |  Lux: ");
    Serial.println(data.lux, 2);
  }

  delay(200);
}