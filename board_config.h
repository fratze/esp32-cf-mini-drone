#pragma once

// Board hardware settings
// The build target selects the settings for ESP32, ESP32-C3 or ESP32-S3.

#ifdef CONFIG_IDF_TARGET_ESP32C3
// ---------------------- ESP32C3 ------------------------- 

// ---- Motor pins in motors.ino order (rear left, rear right, front right, front left) ----
// GPIO0/1/3/10 have no built-in pull-ups. The C3 boot strapping pins are GPIO2/8/9.
#define BOARD_MOTOR_PINS   {3, 10, 0, 1}    // RL=GPIO3, RR=GPIO10, FR=GPIO0, FL=GPIO1

// ---- Battery ADC ----
// GPIO2 is a boot strapping pin with a weak pull-up. The pull-up does not affect ADC measurements in input mode.
// After startup, analogRead() uses it as a normal input.
#define BOARD_VBAT_ADC_PIN 2               // ADC1_CH2 (GPIO2)

// ---- RC serial port. The default is CRSF/ELRS. Set RC_PROTOCOL to 0 for SBUS. ----
// GPIO9 receives standard CRSF levels without an inverter.
#define BOARD_RC_SERIAL    Serial1
#define BOARD_RC_RX_PIN    9
#define BOARD_RC_TX_PIN    -1          // Reserved for return telemetry. A value of -1 disables it.
#define BOARD_RC_PROTOCOL  1           // 0=SBUS, 1=CRSF(ELRS)
#define BOARD_RC_BAUD      420000      // Default baud rates are 420000 for CRSF and 100000 for SBUS.

// ---- IMU SPI pins from the board documentation and pins_arduino.h ----
// These match the Arduino ESP32-C3 defaults, so SPI.begin() needs no arguments.
#define BOARD_SPI_SCK      4               // Default SCK on GPIO4
#define BOARD_SPI_MISO     5               // Default MISO on GPIO5
#define BOARD_SPI_MOSI     6               // Default MOSI on GPIO6
#define BOARD_SPI_CS       7               // Default SS on GPIO7

// ---- Default ESP32-C3 I2C pins ----
#define BOARD_I2C_SDA          (-1)        // GPIO8 is shared with the LED. Disable the LED when using this pin for I2C.
#define BOARD_I2C_SCL          (-1)        // GPIO9

// ---- Blue status LED on GPIO8, active low ----
#define BOARD_LED_ENABLED  1
#define BOARD_LED_PIN      8
#define BOARD_LED_INVERTED 1               // Low turns the LED on.

// ---- Resource settings for the C3 ----
#define BOARD_VBAT_ADC_SAMPLES       8    // Fewer ADC samples reduce blocking time. ESP32 and S3 use 16.
#define BOARD_LOG_DURATION           4    // A 4-second log saves about 22 KB of RAM. ESP32 and S3 use 8 seconds.
#define BOARD_CONSOLE_LINES          20   // A shorter console saves about 7 KB of RAM. ESP32 and S3 keep 50 lines.
#define BOARD_CONSOLE_LINE_LEN       160  // Characters per line. ESP32 and S3 use 240.
#define BOARD_MAVLINK_TELEM_FAST_HZ  5    // A lower telemetry rate reduces Wi-Fi traffic. ESP32 and S3 use 10 Hz.
#define BOARD_WIFI_ENABLED           0    // Wi-Fi shares the C3's single CPU with flight control. Disable it to free CPU time.
#define BOARD_WEB_RC_ENABLED         0    // The web remote needs Wi-Fi, so disable both together.

// ---- Expansion board sensors ----
#undef  EXPANSION_BOARD_ENABLED
#define EXPANSION_BOARD_ENABLED      0    // Disable the expansion board on C3 because its pins conflict.
#define BOARD_VL53_XSHUT_PIN   (-1)        // VL53L1X XSHUT, tentatively assigned to GPIO10
#define BOARD_PMW_CS_PIN       (-1)        // PMW3901 CS has no C3 pin assignment. A value of -1 disables it.

#elif defined(CONFIG_IDF_TARGET_ESP32S3)
// ---------------------- ESP32S3 -------------------------

// ---- Motor pins GPIO4-7 are separate from FSPI (GPIO10-13) and the boot strapping pins. ----
#define BOARD_MOTOR_PINS   {4, 5, 6, 7}    // RL=GPIO4, RR=GPIO5, FR=GPIO6, FL=GPIO7

// ---- Battery ADC on GPIO1 (ADC1_CH0). S3 ADC1 covers GPIO1-10. ----
#define BOARD_VBAT_ADC_PIN 1

// ---- RC serial input on GPIO8 (CRSF/ELRS by default), separate from FSPI and UART0 ----
#define BOARD_RC_SERIAL    Serial2
#define BOARD_RC_RX_PIN    8
#define BOARD_RC_TX_PIN    -1
#define BOARD_RC_PROTOCOL  1
#define BOARD_RC_BAUD      420000

// ---- Default ESP32-S3 FSPI pins for the IMU ----
// Default pins are SCK 12, MISO 13, MOSI 11 and CS 10.
#define BOARD_SPI_SCK      12
#define BOARD_SPI_MISO     13
#define BOARD_SPI_MOSI     11
#define BOARD_SPI_CS       10

// ---- ESP32-S3 I2C pins ----
#define BOARD_I2C_SDA          35          // GPIO35
#define BOARD_I2C_SCL          36          // GPIO36

// ---- The new PCB uses a standard LED on GPIO2, driven the same way as on ESP32. ----
#define BOARD_LED_ENABLED  1
#define BOARD_LED_PIN      2
#define BOARD_LED_INVERTED 0

// ---- Standard S3 resource settings ----
#define BOARD_VBAT_ADC_SAMPLES       16 // ADC samples per reading
#define BOARD_LOG_DURATION           8  // Seconds of log data to keep
#define BOARD_CONSOLE_LINES          50  // Number of console lines
#define BOARD_CONSOLE_LINE_LEN       240  // Characters per line
#define BOARD_MAVLINK_TELEM_FAST_HZ  10  // Fast MAVLink telemetry rate in Hz
#define BOARD_WIFI_ENABLED           1
#define BOARD_WEB_RC_ENABLED         1

// ---- Expansion board sensors ----
#define EXPANSION_BOARD_ENABLED      1 // Set to 1 to probe BMP388, VL53L1X, QMC5883L and PMW3901. With 0, sensor flags stay false and basic flight control still works.
#define BOARD_VL53_XSHUT_PIN   37      // VL53L1X XSHUT, tentatively assigned to GPIO37
#define BOARD_PMW_CS_PIN       38      // PMW3901 CS, tentatively assigned to GPIO38

#else  // ---- Default ESP32 configuration ----
// ---------------------- ESP32 defaults -------------------------

#define BOARD_MOTOR_PINS   {12, 13, 15, 14}  // Motor pins RL=GPIO12, RR=GPIO13, FR=GPIO15, FL=GPIO14

#define BOARD_VBAT_ADC_PIN 36              // Battery voltage ADC on GPIO36 (input only)

#define BOARD_RC_SERIAL    Serial2         // RC serial port (CRSF/ELRS by default)
#define BOARD_RC_RX_PIN    4               // RC input on GPIO4
#define BOARD_RC_TX_PIN    -1              // Reserved for return telemetry. A value of -1 disables it.
#define BOARD_RC_PROTOCOL  1               // 0=SBUS, 1=CRSF(ELRS)
#define BOARD_RC_BAUD      420000          // Default baud rates are 420000 for CRSF and 100000 for SBUS.

#define BOARD_SPI_SCK      18              // SPI clock on GPIO18
#define BOARD_SPI_MISO     19              // SPI MISO on GPIO19
#define BOARD_SPI_MOSI     23              // SPI MOSI on GPIO23
#define BOARD_SPI_CS       5               // SPI chip select on GPIO5

#define BOARD_I2C_SDA      21              // GPIO21 is the default ESP32 I2C SDA pin.
#define BOARD_I2C_SCL      22              // GPIO22 is the default ESP32 I2C SCL pin.

#define BOARD_LED_ENABLED  1               // Enable the onboard LED.
#define BOARD_LED_PIN      2               // LED on GPIO2
#define BOARD_LED_INVERTED 0               // High turns the LED on.

// ---- Standard ESP32 resource settings ----
#define BOARD_VBAT_ADC_SAMPLES       16 // ADC samples per reading
#define BOARD_LOG_DURATION           8  // Seconds of log data to keep
#define BOARD_CONSOLE_LINES          50  // Number of console lines
#define BOARD_CONSOLE_LINE_LEN       240  // Characters per line
#define BOARD_MAVLINK_TELEM_FAST_HZ  10  // Fast MAVLink telemetry rate in Hz
#define BOARD_WIFI_ENABLED           1  // Enable Wi-Fi.
#define BOARD_WEB_RC_ENABLED         1  // Enable the web remote.

// ---- Expansion board sensors ----
#define EXPANSION_BOARD_ENABLED      1 // Set to 1 to probe BMP388, VL53L1X, QMC5883L and PMW3901. With 0, sensor flags stay false and basic flight control still works.
#define BOARD_VL53_XSHUT_PIN   32          // VL53L1X XSHUT (GPIO32)
#define BOARD_PMW_CS_PIN       33          // PMW3901 SPI CS (GPIO33)

#endif

// ---- Fallback settings for other chips ----

// ---- Expansion board switch ----
// Set to 1 to probe BMP388, VL53L1X, QMC5883L and PMW3901.
// Set to 0 to skip detection. Sensor flags stay false and basic flight control still works.
#if !defined(EXPANSION_BOARD_ENABLED)
#define EXPANSION_BOARD_ENABLED 1  // Enable by default on other chips.
#endif

// ---- Enable Wi-Fi and the web remote by default on other chips. ----
#if !defined(BOARD_WIFI_ENABLED)
#define BOARD_WIFI_ENABLED   1
#endif
#if !defined(BOARD_WEB_RC_ENABLED)
#define BOARD_WEB_RC_ENABLED 1
#endif
