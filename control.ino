// Flight control

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "lpf.h"
#include "util.h"

#if WEB_RC_ENABLED
#include "control.h"
#endif

// These settings are for a small quadcopter with a 118 mm wheelbase.
// ============== Inner loop (angular rate) ==============
#define PITCHRATE_P 0.06 // Increasing P gives a faster response.
#define PITCHRATE_I 0.1 // Moderate I compensates for differences between motors.
#define PITCHRATE_D 0.001 // A small D term damps oscillation.
#define PITCHRATE_I_LIM 0.3 // Limit the accumulated integral.
#define ROLLRATE_P PITCHRATE_P // Roll and pitch use the same settings.
#define ROLLRATE_I PITCHRATE_I 
#define ROLLRATE_D PITCHRATE_D
#define ROLLRATE_I_LIM PITCHRATE_I_LIM
#define YAWRATE_P 0.3 // Yaw uses higher P because its inertia is lower.
#define YAWRATE_I 0.01 // Moderate integral gain
#define YAWRATE_D 0.01 // Small derivative gain
#define YAWRATE_I_LIM 0.3
// ============== Outer loop (angle) ==============
#define ROLL_P 6 // Higher P gives a faster response.
#define ROLL_I 0 // Outer-loop integral gain
#define ROLL_D 0 // The angle loop usually needs no derivative term.
#define ROLL_I_LIM radians(5.0f) // Limit the roll integral to about 5°/s (in rad/s) to prevent buildup at low throttle or during mode changes.
#define PITCH_P ROLL_P // Use the same settings for roll and pitch.
#define PITCH_I ROLL_I
#define PITCH_D ROLL_D
#define PITCH_I_LIM ROLL_I_LIM
#define YAW_P 3 // Yaw responds a little more slowly.

// ============== Limits ==============
#define PITCHRATE_MAX radians(360) // Maximum pitch rate (360°/s)
#define ROLLRATE_MAX radians(360)
#define YAWRATE_MAX radians(300) // Lower rotation rate for yaw
#define TILT_MAX radians(30) // Maximum tilt of 30°
#define ALTHOLD_HOVER_THRUST 0.5f   // Base hover thrust for altitude hold until barometer support is available
#define ARM_THROTTLE_LIMIT   0.05f  // Block arming above 5% throttle (normalized range 0 to 1).
#define RATES_D_LPF_ALPHA 0.2 // cutoff frequency ~ 40 Hz

float motThrMin = 0.10f;  // Output at minimum stick position, set by MOT_THR_MIN
float motThrMax = 0.9f;   // Output at maximum stick position, set by MOT_THR_MAX. Leave 10% for attitude corrections.

const int RAW = 0, ACRO = 1, STAB = 2, ALTHOLD = 3, AUTO = 4; // flight modes
int mode = STAB;
bool armed = false;
int flightModes[] = {STAB, STAB, STAB}; // CTL_FLT_MODE_0/1/2 set the flight modes for the three switch positions.

#if WEB_RC_ENABLED
extern uint16_t webRCButtons;
#endif

PID rollRatePID(ROLLRATE_P, ROLLRATE_I, ROLLRATE_D, ROLLRATE_I_LIM, RATES_D_LPF_ALPHA);
PID pitchRatePID(PITCHRATE_P, PITCHRATE_I, PITCHRATE_D, PITCHRATE_I_LIM, RATES_D_LPF_ALPHA);
PID yawRatePID(YAWRATE_P, YAWRATE_I, YAWRATE_D);
PID rollPID(ROLL_P, ROLL_I, ROLL_D, ROLL_I_LIM);
PID pitchPID(PITCH_P, PITCH_I, PITCH_D, PITCH_I_LIM);
PID yawPID(YAW_P, 0, 0);
Vector maxRate(ROLLRATE_MAX, PITCHRATE_MAX, YAWRATE_MAX);
float tiltMax = TILT_MAX;

Quaternion attitudeTarget;
Vector ratesTarget;
Vector ratesExtra; // feedforward rates
Vector torqueTarget;
float thrustTarget;

// ============== Software trim ==============
// Trim corrects steady drift from uneven weight, motor or propeller differences, 
// or IMU alignment.
// Angles are in radians.
//
// Adjust trim in the serial or web console without flashing the firmware again.
// Command format: p <name> <value>
// Adjust in steps of 0.005 rad (about 0.3°) until drift stops with the sticks centered.
// Settings are saved to flash and survive a restart.
//
// Release the sticks and observe the direction of drift.
// Left drift: p CTL_TRIM_ROLL +0.01 tilts right.
// Right drift: p CTL_TRIM_ROLL -0.01 tilts left.
// Forward drift: p CTL_TRIM_PITCH -0.01 raises the nose.
// Backward drift: p CTL_TRIM_PITCH +0.01 lowers the nose.
//
// Changes in battery level or payload may require a small trim adjustment.
float trimRoll  = 0.0f; // Roll trim in radians (CTL_TRIM_ROLL)
float trimPitch = 0.0f; // Pitch trim in radians (CTL_TRIM_PITCH)

extern const int MOTOR_REAR_LEFT, MOTOR_REAR_RIGHT, MOTOR_FRONT_RIGHT, MOTOR_FRONT_LEFT;
extern float motors[4];
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern float batteryVoltage;  // battery.ino

void control() {
	interpretControls();
#if WEB_RC_ENABLED
	interpretWebRC();
#endif
	failsafe();
	controlAttitude();
	controlRates();
	controlTorque();
}

void interpretControls() {
	if (controlMode < 0.25) mode = flightModes[0];
	else if (controlMode <= 0.75) mode = flightModes[1];
	else if (controlMode > 0.75) mode = flightModes[2];

	if (mode == AUTO) return; // pilot is not effective in AUTO mode

#if WEB_RC_ENABLED
	if (!isUsingWebRC()) { // Allow SBUS stick arming only when the web remote is inactive.
#endif
	extern bool imuOK;
	static bool armWarnNotified = false;  // Limit repeated low-battery and IMU arming warnings.
	if (controlThrottle < 0.05 && controlYaw > 0.95) { // arm gesture
		if (!imuOK) {
			// Block arming on IMU failure.
			if (!armWarnNotified) {
				print("Cannot arm: IMU fault.\n");
#if WEB_RC_ENABLED
				setWebRCWarn("Cannot arm: IMU fault.");
#endif
				armWarnNotified = true;
			}
		} else if (batteryVoltage > VBAT_ABSENT_THRESHOLD && batteryVoltage < VBAT_WARN_THRESHOLD) {
			// At or below L1, block arming and report a change in status.
			if (!armWarnNotified) {
				print("Cannot arm: low battery (%.2f V).\n", batteryVoltage);
#if WEB_RC_ENABLED
				char warnBuf[64];
				snprintf(warnBuf, sizeof(warnBuf), "Cannot arm: low battery (%.2f V).", batteryVoltage);
				setWebRCWarn(warnBuf);
#endif
				armWarnNotified = true;
			}
		} else {
			armed = true;
			armWarnNotified = false; // Reset after arming.
		}
	}
	if (controlThrottle < 0.05 && controlYaw < -0.95) armed = false; // disarm gesture
#if WEB_RC_ENABLED
	}
#endif

	if (abs(controlYaw) < 0.1) controlYaw = 0; // yaw dead zone

	if (controlThrottle < 0.05f) {
		thrustTarget = 0.0f;   // Use idle thrust in the low-end dead zone and skip PID control.
	} else {
		thrustTarget = mapf(controlThrottle, 0.05f, 1.0f, motThrMin, motThrMax);
	}

	if (mode == STAB) {
		float yawTarget = attitudeTarget.getYaw();
		if (!armed || invalid(yawTarget) || controlYaw != 0) yawTarget = attitude.getYaw(); // reset yaw target
		// Add trim to the stick commands to correct steady drift.
		// See the trim settings above. 
		// Use the p command to change CTL_TRIM_ROLL or CTL_TRIM_PITCH.
		attitudeTarget = Quaternion::fromEuler(Vector(controlRoll * tiltMax + trimRoll, controlPitch * tiltMax + trimPitch, yawTarget));
		ratesExtra = Vector(0, 0, -controlYaw * maxRate.z); // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == ACRO) {
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.x = controlRoll * maxRate.x;
		ratesTarget.y = controlPitch * maxRate.y;
		ratesTarget.z = -controlYaw * maxRate.z; // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == RAW) { // direct torque control
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.invalidate(); // skip rate control
		torqueTarget = Vector(controlRoll, controlPitch, -controlYaw) * 0.1;
	}
}

void controlAttitude() {
	if (!armed || attitudeTarget.invalid() || thrustTarget < motThrMin) return; // skip attitude control

	const Vector up(0, 0, 1);
	Vector upActual = Quaternion::rotateVector(up, attitude);
	Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);

	Vector error = Vector::rotationVectorBetween(upTarget, upActual);

	ratesTarget.x = rollPID.update(error.x) + ratesExtra.x;
	ratesTarget.y = pitchPID.update(error.y) + ratesExtra.y;

	float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
	ratesTarget.z = yawPID.update(yawError) + ratesExtra.z;
}


void controlRates() {
	if (!armed || ratesTarget.invalid() || thrustTarget < motThrMin) return; // skip rates control

	Vector error = ratesTarget - rates;

	// Calculate desired torque, where 0 - no torque, 1 - maximum possible torque
	torqueTarget.x = rollRatePID.update(error.x);
	torqueTarget.y = pitchRatePID.update(error.y);
	torqueTarget.z = yawRatePID.update(error.z);
}

void controlTorque() {
	if (!torqueTarget.valid()) return; // skip torque control

	if (!armed) {
		memset(motors, 0, sizeof(motors)); // stop motors if disarmed
		return;
	}

	if (thrustTarget < motThrMin) {
		for (int i = 0; i < 4; i++) motors[i] = motThrMin; // idle thrust
		return;
	}

	motors[MOTOR_FRONT_LEFT] = thrustTarget + torqueTarget.x - torqueTarget.y + torqueTarget.z;
	motors[MOTOR_FRONT_RIGHT] = thrustTarget - torqueTarget.x - torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_LEFT] = thrustTarget + torqueTarget.x + torqueTarget.y - torqueTarget.z;
	motors[MOTOR_REAR_RIGHT] = thrustTarget - torqueTarget.x + torqueTarget.y + torqueTarget.z;

	desaturate(motors[MOTOR_FRONT_LEFT], motors[MOTOR_FRONT_RIGHT], motors[MOTOR_REAR_LEFT], motors[MOTOR_REAR_RIGHT]);

	motors[0] = constrain(motors[0], 0, 1);
	motors[1] = constrain(motors[1], 0, 1);
	motors[2] = constrain(motors[2], 0, 1);
	motors[3] = constrain(motors[3], 0, 1);
}

void desaturate(float& a, float& b, float& c, float& d) {
	// Torque terms sum to zero, so the average is close to thrustTarget.
	// Scale deviations while keeping the average thrust.
	float avg = (a + b + c + d) * 0.25f;
	float maxVal = max(max(a, b), max(c, d));
	float minVal = min(min(a, b), min(c, d));

	float scale = 1.0f;
	if (maxVal > 1.0f && maxVal > avg) {
		scale = min(scale, (1.0f - avg) / (maxVal - avg));
	}
	if (minVal < 0.0f && avg > minVal) {
		scale = min(scale, avg / (avg - minVal));
	}

	if (scale < 1.0f) {
		a = avg + (a - avg) * scale;
		b = avg + (b - avg) * scale;
		c = avg + (c - avg) * scale;
		d = avg + (d - avg) * scale;
	}
}

const char* getModeName() {
	switch (mode) {
		case RAW:     return "RAW";
		case ACRO:    return "ACRO";
		case STAB:    return "STAB";
		case ALTHOLD: return "ALTHOLD";
		case AUTO:    return "AUTO";
		default:      return "UNKNOWN";
	}
}

#if WEB_RC_ENABLED
// Web remote buttons for arming, disarming and mode changes
void interpretWebRC() {
	if (!isUsingWebRC()) return;

	// Handle button presses on the rising edge so each press acts only once.
	static uint16_t lastWebRCButtons = 0;
	uint16_t risingEdge = webRCButtons & ~lastWebRCButtons;
	lastWebRCButtons = webRCButtons;

	// Log changes in armed state.
	static bool lastArmedState = false;
	if (armed != lastArmedState) {
		print(armed ? "Web RC: armed\n" : "Web RC: disarmed\n");
		lastArmedState = armed;
	}

	// Button 0 pressed: arm.
	if (risingEdge & 0x0001) {
		extern bool imuOK;
		extern char webRCWarnMsg[];
		if (!imuOK) {
			setWebRCWarn("Cannot arm: IMU fault.");
		} else if (batteryVoltage > VBAT_ABSENT_THRESHOLD && batteryVoltage < VBAT_WARN_THRESHOLD) {
			// Block arming on low battery, as in the SBUS path in interpretControls().
			char warnBuf[64];
			snprintf(warnBuf, sizeof(warnBuf), "Cannot arm: low battery (%.2f V).", batteryVoltage);
			setWebRCWarn(warnBuf);
		} else if (controlThrottle > ARM_THROTTLE_LIMIT) {
			setWebRCWarn("Lower the throttle before arming.");
		} else {
			armed = true;
			webRCWarnMsg[0] = '\0'; // Clear the previous warning after arming.
		}
	}

	// Button 1 pressed: disarm.
	if (risingEdge & 0x0002) {
		armed = false;
	}

	// Button 2 pressed: emergency stop.
	if (risingEdge & 0x0004) {
		armed = false;
		thrustTarget = 0.0f;
	}

	// Button 6 pressed: stabilize mode.
	if (risingEdge & 0x0040) {
		mode = STAB;
	}

	// Button 7 pressed: acro mode.
	if (risingEdge & 0x0080) {
		mode = ACRO;
	}

	// Button 8 pressed: altitude hold is unavailable. Return to stabilize so the web remote can keep cycling modes.
	if (risingEdge & 0x0100) {
		mode = STAB;
		setWebRCWarn("Altitude hold unavailable. Switched to stabilize.");
	}

	// Log mode changes.
	static int lastMode = STAB;
	if (mode != lastMode) {
		print("Web RC: switched to %s\n", getModeName());
		lastMode = mode;
	}
}
#endif
