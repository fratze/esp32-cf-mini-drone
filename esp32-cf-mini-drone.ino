// Main sketch
// Build with Arduino IDE 2.3.x or later. IDE 1.8.19 is unsupported.
// Install "esp32 by Espressif Systems" and the MAVLink library.

#include "vector.h"
#include "quaternion.h"
#include "util.h"
#include "board_config.h"

// board_config.h sets Wi-Fi and web remote support for each chip.
// To override a setting here, use #undef before #define.
#define WIFI_ENABLED    BOARD_WIFI_ENABLED
#define WEB_RC_ENABLED  BOARD_WEB_RC_ENABLED

float t = NAN; // Current loop time in seconds
float dt; // Seconds since the previous loop
float controlRoll, controlPitch, controlYaw, controlThrottle; // Pilot commands in the range [-1, 1]
float controlMode = NAN;
Vector gyro; // Gyroscope readings
Vector acc; // Acceleration in m/s²
Vector rates; // Filtered angular velocity in rad/s
Quaternion attitude; // Estimated orientation as a quaternion
bool landed; // are we landed and stationary

void setup() {
	Serial.begin(115200); // Start the serial port at 115200 baud.
	disableBrownOut(); // Disable brownout resets so a brief voltage dip at motor startup does not reset the board.
	print("CHERGLOW CF-Drone Flight Controller\n"); // Article 2 of LICENSE requires the brand name in the startup message.
	print("Starting flight controller...\n");
	setupParameters(); // Load saved settings from flash, or use the defaults.
	setupLED(); // Set up the status LED.
	setupMotors(); // Set up the motor outputs.
	setLED(true); // Turn on the LED during setup.
#if WIFI_ENABLED
	setupWiFi(); // Start Wi-Fi for the web remote and MAVLink.
#endif
#if WEB_RC_ENABLED
	setupWebRC();  // Start the web remote.
#endif
	setupIMU(); // Start the gyroscope and accelerometer.
	setupRC(); // Set up the radio receiver (SBUS or ELRS).
	setLED(false); // Turn off the LED when setup is complete.
	print("Flight controller ready.\n");
	print("================================\n");
}

void loop() {
	readIMU(); // Read the IMU, apply calibration and rotate the readings into the frame used by the controller.
	step(); // Update loop time, elapsed time and loop frequency.
	readRC(); // Read the radio receiver.
#if WEB_RC_ENABLED
	readWebRC();  // Read the web remote.
	processConsoleCommandQueue(); // Run queued web commands here to keep HTTP callbacks short.
#endif
	estimate(); // Combine IMU readings to estimate orientation and motion.
	updateBatteryVoltage(); // Sample the battery voltage and check the low-voltage thresholds.
	control(); // Run the attitude controller and calculate motor commands.
	sendMotors(); // Send the commands to the motors.
	handleInput(); // Handle serial and web console commands.
#if WIFI_ENABLED
	processMavlink(); // Process MAVLink messages.
#endif
	logData(); // Record flight data.
	syncParameters(); // Wait before saving changes to reduce flash wear.
	updateLED(); // Update the status LED for the current flight state.
}
