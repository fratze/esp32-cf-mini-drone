// Store parameters in flash.

#include <Preferences.h>
#include <nvs_flash.h> // Erase and reinitialize NVS if Preferences.begin() fails.
#include "util.h"

extern float channelZero[16];
extern float channelMax[16];
extern float rollChannel, pitchChannel, throttleChannel, yawChannel, modeChannel;
extern int motorPins[4];
extern int pwmFrequency, pwmResolution, pwmStop, pwmMin, pwmMax;
extern float motThrMin;
extern float motThrMax;
#if WIFI_ENABLED
extern int wifiMode, udpLocalPort, udpRemotePort;
extern int mavlinkSysId;
extern Rate telemetrySlow, telemetryFast;
#endif
extern float rcLossTimeout, descendTime;
extern int flightModes[3];
extern Vector accBias, accScale;
extern Vector imuRotation;
extern LowPassFilter<Vector> gyroBiasFilter;
extern int rcRxPin;
extern int rcProtocol;   // RC protocol (0 = SBUS, 1 = CRSF/ELRS)
extern int rcTxPin;      // Telemetry TX pin (-1 disables it)
extern int rcBaud;       // Serial baud rate
extern float trimRoll, trimPitch;         // Trim settings from control.ino
extern float levelGateThreshold;         // Stick threshold from estimate.ino
extern float levelBiasGain;              // Mahony integral gain from estimate.ino
extern Vector levelGyroBias;             // Estimated Mahony gyro bias from estimate.ino

Preferences storage;

struct Parameter {
	const char *name; // max length is 15 (Preferences key limit)
	bool integer;
	union { float *f; int *i; };
	float cache; // what's stored in flash
	void (*callback)(); // called after parameter change
	Parameter(const char *name, float *variable, void (*callback)() = nullptr) : name(name), integer(false), f(variable), cache(0), callback(callback) {};
	Parameter(const char *name, int *variable, void (*callback)() = nullptr) : name(name), integer(true), i(variable), cache(0), callback(callback) {};
	float getValue() const { return integer ? *i : *f; }
	void setValue(const float value) { if (integer) *i = (int)value; else *f = value; }
};

Parameter parameters[] = {
	// ===== Inner angular-rate PID loop =====
	// The inner loop controls how fast the aircraft rotates.
	// P sets response speed. Too much causes oscillation.
	// I removes steady errors such as motor imbalance. Too much can cause windup.
	// D reduces overshoot. Too much amplifies high-frequency noise.
	// WU limits how far the integral term can build up.
	{"CTL_R_RATE_P",  &rollRatePID.p},      // Roll rate P gain (rad/s to torque)
	{"CTL_R_RATE_I",  &rollRatePID.i},      // Roll rate I gain
	{"CTL_R_RATE_D",  &rollRatePID.d},      // Roll rate D gain
	{"CTL_R_RATE_WU", &rollRatePID.windup}, // Roll rate integral limit (±windup)
	{"CTL_P_RATE_P",  &pitchRatePID.p},     // Pitch rate P gain
	{"CTL_P_RATE_I",  &pitchRatePID.i},     // Pitch rate I gain
	{"CTL_P_RATE_D",  &pitchRatePID.d},     // Pitch rate D gain
	{"CTL_P_RATE_WU", &pitchRatePID.windup},// Pitch rate integral limit
	{"CTL_Y_RATE_P",  &yawRatePID.p},       // Yaw rate P gain (lower yaw inertia usually needs a higher value)
	{"CTL_Y_RATE_I",  &yawRatePID.i},       // Yaw rate I gain
	{"CTL_Y_RATE_D",  &yawRatePID.d},       // Yaw rate D gain

	// ===== Outer angle PID loop =====
	// The outer loop converts angle error into a rate command for the inner loop.
	// The integral term corrects a steady hover offset. Set its WU limit to enable it.
	{"CTL_R_P",  &rollPID.p},      // Roll angle P gain (angle error in rad to rate in rad/s)
	{"CTL_R_I",  &rollPID.i},      // Roll angle I gain for steady hover offset
	{"CTL_R_D",  &rollPID.d},      // Roll angle D gain (usually 0)
	{"CTL_R_WU", &rollPID.windup}, // Roll integral limit (rad/s). Set this when I is nonzero to prevent windup.
	{"CTL_P_P",  &pitchPID.p},     // Pitch angle P gain
	{"CTL_P_I",  &pitchPID.i},     // Pitch angle I gain
	{"CTL_P_D",  &pitchPID.d},     // Pitch angle D gain (usually 0)
	{"CTL_P_WU", &pitchPID.windup},// Pitch integral limit (rad/s)
	{"CTL_Y_P",  &yawPID.p},       // Yaw angle P gain

	// ===== Control limits =====
	{"CTL_P_RATE_MAX", &maxRate.y}, // Maximum pitch rate (rad/s). Stick commands are capped at this value.
	{"CTL_R_RATE_MAX", &maxRate.x}, // Maximum roll rate (rad/s)
	{"CTL_Y_RATE_MAX", &maxRate.z}, // Maximum yaw rate (rad/s)
	{"CTL_TILT_MAX",   &tiltMax},   // Maximum tilt in stabilize mode (rad). Limits the outer angle loop.

	// ===== Software trim =====
	// Trim corrects steady hover drift from uneven weight or motor thrust.
	{"CTL_TRIM_ROLL",  &trimRoll},  // Roll trim (rad). Positive tilts right. Suggested step is 0.005 rad.
	{"CTL_TRIM_PITCH", &trimPitch}, // Pitch trim (rad). Suggested step is 0.005 rad. See control.ino for the direction of correction.

	// ===== Flight modes =====
	// RC_MODE maps each of the three switch positions to a flight mode.
	// 0 = RAW (direct), 1 = ACRO, 2 = STAB (stabilize), 3 = ALTHOLD (barometric altitude hold), 4 = AUTO
	{"CTL_FLT_MODE_0", &flightModes[0]}, // Flight mode for switch position 0
	{"CTL_FLT_MODE_1", &flightModes[1]}, // Flight mode for switch position 1
	{"CTL_FLT_MODE_2", &flightModes[2]}, // Flight mode for switch position 2

	// ===== IMU mounting orientation =====
	// Set the mounting angles in radians if the IMU axes differ from the aircraft axes.
	{"IMU_ROT_ROLL",  &imuRotation.x}, // IMU mounting angle about the body X (roll) axis in rad
	{"IMU_ROT_PITCH", &imuRotation.y}, // IMU mounting angle about the body Y (pitch) axis in rad
	{"IMU_ROT_YAW",   &imuRotation.z}, // IMU mounting angle about the body Z (yaw) axis in rad

	// ===== Gyro bias filter =====
	// At startup, estimate stationary gyro bias with an exponential moving average.
	// Lower alpha gives more smoothing but takes longer to settle. Higher alpha responds faster to vibration.
	{"IMU_GYRO_BIAS_A", &gyroBiasFilter.alpha}, // Gyro bias filter coefficient alpha (0 to 1)

	// ===== Accelerometer calibration =====
	// Perform six-face calibration on a new or remounted sensor. These parameters store the results.
	// Calibration formula: acc_corrected = (acc_raw - bias) * scale
	{"IMU_ACC_BIAS_X", &accBias.x},  // Accelerometer X offset (m/s²)
	{"IMU_ACC_BIAS_Y", &accBias.y},  // Accelerometer Y offset (m/s²)
	{"IMU_ACC_BIAS_Z", &accBias.z},  // Accelerometer Z offset (m/s²)
	{"IMU_ACC_SCALE_X", &accScale.x}, // Accelerometer X scale factor (unitless, ideally 1.0)
	{"IMU_ACC_SCALE_Y", &accScale.y}, // Accelerometer Y scale factor
	{"IMU_ACC_SCALE_Z", &accScale.z}, // Accelerometer Z scale factor

	// ===== Attitude estimation =====
	// A complementary filter tracks fast motion with the gyro and uses gravity for slow correction.
	{"EST_ACC_WEIGHT",   &accWeight},          // Accelerometer correction weight (0 to 1). Higher values increase correction and vibration sensitivity.
	{"EST_RATES_LPF_A",  &ratesFilter.alpha},  // Rate filter alpha (0 to 1). Lower values lower the cutoff (about 40 Hz at 0.2).
	{"EST_LVL_GATE_THR", &levelGateThreshold}, // Stick threshold (0 to 1). Fade applyLevel correction to zero by this point to reduce drift after stick release.
	{"EST_LVL_BIAS_GAIN",&levelBiasGain},      // Mahony integral gain sets how fast gravity error changes gyro bias. Settling takes about 30 s at 0.00002.

	// ===== Motor pins =====
	// Pin changes call setupMotors() immediately to reconfigure LEDC without restarting.
	{"MOT_PIN_FL", &motorPins[MOTOR_FRONT_LEFT],  setupMotors}, // Front left motor GPIO
	{"MOT_PIN_FR", &motorPins[MOTOR_FRONT_RIGHT], setupMotors}, // Front right motor GPIO
	{"MOT_PIN_RL", &motorPins[MOTOR_REAR_LEFT],   setupMotors}, // Rear left motor GPIO
	{"MOT_PIN_RR", &motorPins[MOTOR_REAR_RIGHT],  setupMotors}, // Rear right motor GPIO

	// ===== Motor PWM settings =====
	// For direct MOSFET drive use duty-cycle control with pwmMax=-1 and pwmStop=pwmMin=0.
	// For an ESC, set pwmMax and pwmMin to the pulse-width limits in microseconds and use 400 Hz.
	{"MOT_PWM_FREQ", &pwmFrequency, setupMotors}, // PWM frequency (Hz). Suggested values are 25000 for MOSFET drive and 400 for an ESC.
	{"MOT_PWM_RES",  &pwmResolution, setupMotors},// PWM resolution in bits (usually 10). The maximum LEDC count is 2^n - 1.
	{"MOT_PWM_STOP", &pwmStop},                   // Stopped motor output. Use 0 in duty-cycle mode or the idle pulse width in microseconds for an ESC.
	{"MOT_PWM_MIN",  &pwmMin},                    // Minimum output. Use 0 in duty-cycle mode or the minimum ESC pulse width in microseconds.
	{"MOT_PWM_MAX",  &pwmMax},                    // Maximum output. Use -1 for MOSFET duty-cycle mode or the maximum ESC pulse width in microseconds.

	// ===== Motor thrust range =====
	// Map throttle (0 to 1) linearly into the motor output range and leave room for attitude corrections.
	{"MOT_THR_MIN", &motThrMin}, // Output at minimum throttle (0 to 1). At least 0.05 is suggested to avoid stalls.
	{"MOT_THR_MAX", &motThrMax}, // Output at maximum throttle (0 to 1). At most 0.9 is suggested to leave room for corrections.

	// ===== Radio channel calibration =====
	// Calibration converts raw radio readings to -1 to 1 or 0 to 1.
	// channelZero[n] stores the neutral reading. channelMax[n] stores the reading at full travel.
	// Calibration formula: normalized = (raw - zero) / (max - zero)
	{"RC_ZERO_0", &channelZero[0]}, // Channel 0 neutral
	{"RC_ZERO_1", &channelZero[1]}, // Channel 1 neutral
	{"RC_ZERO_2", &channelZero[2]}, // Channel 2 neutral
	{"RC_ZERO_3", &channelZero[3]}, // Channel 3 neutral
	{"RC_ZERO_4", &channelZero[4]}, // Channel 4 neutral
	{"RC_ZERO_5", &channelZero[5]}, // Channel 5 neutral
	{"RC_ZERO_6", &channelZero[6]}, // Channel 6 neutral
	{"RC_ZERO_7", &channelZero[7]}, // Channel 7 neutral
	{"RC_MAX_0",  &channelMax[0]},  // Channel 0 travel
	{"RC_MAX_1",  &channelMax[1]},  // Channel 1 travel
	{"RC_MAX_2",  &channelMax[2]},  // Channel 2 travel
	{"RC_MAX_3",  &channelMax[3]},  // Channel 3 travel
	{"RC_MAX_4",  &channelMax[4]},  // Channel 4 travel
	{"RC_MAX_5",  &channelMax[5]},  // Channel 5 travel
	{"RC_MAX_6",  &channelMax[6]},  // Channel 6 travel
	{"RC_MAX_7",  &channelMax[7]},  // Channel 7 travel

	// ===== Radio channel assignments =====
	// Choose the physical channel for each control, counting from zero.
	{"RC_ROLL",     &rollChannel},     // Roll channel
	{"RC_PITCH",    &pitchChannel},    // Pitch channel
	{"RC_THROTTLE", &throttleChannel}, // Throttle channel
	{"RC_YAW",      &yawChannel},      // Yaw channel
	{"RC_MODE",     &modeChannel},     // Three-position flight mode switch channel

	// ===== Receiver serial port settings =====
	// Changing any of these four parameters calls setupRC() immediately to reconfigure the port.
	{"RC_RX_PIN",   &rcRxPin,    setupRC}, // Receiver input GPIO (UART RX)
	{"RC_PROTOCOL", &rcProtocol, setupRC}, // RC protocol: 0 = SBUS (inverted), 1 = CRSF/ELRS (non-inverted, 420000 baud)
	{"RC_TX_PIN",   &rcTxPin,    setupRC}, // Return telemetry GPIO (UART TX). Use -1 to disable it.
	{"RC_BAUD",     &rcBaud,     setupRC}, // Radio baud rate (100000 for SBUS, 420000 for CRSF)

#if WIFI_ENABLED
	// ===== Wi-Fi settings =====
	{"WIFI_MODE",     &wifiMode},      // Wi-Fi mode: 0 = off, 1 = create an access point, 2 = join an existing network
	{"WIFI_LOC_PORT", &udpLocalPort},  // Local UDP port for messages from the ground station
	{"WIFI_REM_PORT", &udpRemotePort}, // Remote UDP port for messages from the flight controller

	// ===== MAVLink telemetry =====
	{"MAV_SYS_ID",    &mavlinkSysId},       // MAVLink system ID (1 to 254). Use a different ID for each aircraft.
	{"MAV_RATE_SLOW", &telemetrySlow.rate}, // Slow telemetry rate in Hz for heartbeats, battery readings and similar data
	{"MAV_RATE_FAST", &telemetryFast.rate}, // Fast telemetry rate in Hz for attitude and angular rate
#endif

	// ===== Failsafe settings =====
	{"SF_RC_LOSS_TIME",  &rcLossTimeout}, // Seconds without an RC signal before automatic descent
	{"SF_DESCEND_TIME",  &descendTime},   // Seconds to reduce throttle to zero during descent. Lower values descend faster.
};

void setupParameters() {
	print("Setup parameters\n");
	if (!storage.begin("flix", false)) {
		// Preferences.begin() returns false if NVS initialization fails, for example after a partition change, an incompatible NVS version or power loss.
		// Erase the partition and retry so later parameter writes can succeed.
		print("[NVS] Could not open settings. Erasing the NVS partition and trying again...\n");
		nvs_flash_erase();
		esp_err_t err = nvs_flash_init();
		print("[NVS] nvs_flash_init retry returned %d\n", err);
		if (!storage.begin("flix", false)) {
			print("[NVS] Setup failed again. Settings cannot be saved. Check the partition table and hardware.\n");
		} else {
			print("[NVS] Storage rebuilt. Old settings were erased. Defaults are active. Repeat calibration with ca and cr.\n");
		}
	}
	// Read parameters from storage
	for (auto &parameter : parameters) {
		if (!storage.isKey(parameter.name)) {
			storage.putFloat(parameter.name, parameter.getValue()); // store default value
		}
		float stored = storage.getFloat(parameter.name, parameter.getValue());
		// If a damaged flash entry gives NaN for an integer parameter, restore and save the default.
		if (parameter.integer && !isfinite(stored)) {
			stored = (float)parameter.getValue();
			storage.putFloat(parameter.name, stored);
		}
		parameter.setValue(stored);
		parameter.cache = parameter.getValue();
	}
}

int parametersCount() {
	return sizeof(parameters) / sizeof(parameters[0]);
}

const char *getParameterName(int index) {
	if (index < 0 || index >= parametersCount()) return "";
	return parameters[index].name;
}

float getParameter(int index) {
	if (index < 0 || index >= parametersCount()) return NAN;
	return parameters[index].getValue();
}

float getParameter(const char *name) {
	for (auto &parameter : parameters) {
		if (strcasecmp(parameter.name, name) == 0) {
			return parameter.getValue();
		}
	}
	return NAN;
}

bool setParameter(const char *name, const float value) {
	for (auto &parameter : parameters) {
		if (strcasecmp(parameter.name, name) == 0) {
			if (parameter.integer && !isfinite(value)) return false; // can't set integer to NaN or Inf
			parameter.setValue(value);
			if (parameter.callback) parameter.callback();
			return true;
		}
	}
	return false;
}

void syncParameters() {
	static Rate rate(1);
	if (!rate) return; // sync once per second
	if (motorsActive()) return; // don't use flash while flying, it may cause a delay

	for (auto &parameter : parameters) {
		if (parameter.getValue() == parameter.cache) continue;
		if (isnan(parameter.getValue()) && isnan(parameter.cache)) continue; // handle NAN != NAN
		size_t written = storage.putFloat(parameter.name, parameter.getValue());
		if (written != sizeof(float)) continue; // On write failure, keep the old cached value and retry on the next 1 Hz update.
		parameter.cache = parameter.getValue();
	}
}

void printParameters() {
	for (auto &parameter : parameters) {
		print("%s = %g\n", parameter.name, parameter.getValue());
	}
}

void resetParameters() {
	storage.clear();
	ESP.restart();
}
