// Failsafe protection

bool isInverted = false;  // True if the aircraft is upside down (Z-axis cosine below INVERTED_COS_THRESHOLD)

float rcLossTimeout = 1;        // Receiver signal timeout in seconds, set by SF_RC_LOSS_TIME
float descendTime = 10;         // Seconds from descent to motor stop, set by SF_DESCEND_TIME
#define WEB_RC_LOSS_TIMEOUT_MS 8000UL  // Web remote timeout in ms. It must exceed the 2000 ms heartbeat interval.

// Inversion protection settings
#define INVERTED_COS_THRESHOLD -0.7f   // Cosine of 134°. Treat larger tilt angles as inverted, allowing for gyro drift.
#define INVERTED_TIMEOUT        1.5f   // Stop the motors after 1.5 seconds upside down.

extern float controlTime;
extern float controlRoll, controlPitch, controlThrottle, controlYaw;

#if WEB_RC_ENABLED
// Web remote state
extern bool webRCEnabled;
extern bool useWebRC;
extern unsigned long webRCLastUpdate;
bool isUsingWebRC();
#endif

extern bool armed;
extern int mode;
extern float dt;
extern float thrustTarget;
extern float t;
extern float batteryVoltage;  // battery.ino
extern Quaternion attitudeTarget;
extern Quaternion attitude;
extern Vector ratesExtra;      // control.ino
extern PID rollRatePID, pitchRatePID, yawRatePID;  // control.ino
extern PID rollPID, pitchPID, yawPID;              // control.ino
extern float motThrMin;        // control.ino

void failsafe() {
	rcLossFailsafe();
#if WEB_RC_ENABLED
	webRCLossFailsafe();
#endif
	autoFailsafe();
	invertedFailsafe();
	batteryFailsafe();
}

// RC loss failsafe
void rcLossFailsafe() {
	if (controlTime == 0) return; // no RC at all
	if (!armed) return;
#if WEB_RC_ENABLED
	if (isUsingWebRC()) return; // The web remote handles its own timeout in webRCLossFailsafe().
#endif
	if (t - controlTime > rcLossTimeout) {
		descend();
	}
}

// Smooth descend on RC lost
void descend() {
	if (mode != AUTO) {
		// On entry, keep the current yaw, level the aircraft, clear rate feedforward and reset the PID integrals.
		float currentYaw = attitude.getYaw();
		attitudeTarget = Quaternion::fromEuler(Vector(0, 0, currentYaw));
		ratesExtra = Vector(0, 0, 0);
		rollRatePID.reset();
		pitchRatePID.reset();
		yawRatePID.reset();
		rollPID.reset();
		pitchPID.reset();
		yawPID.reset();
		// Limit thrust to hover thrust so high initial throttle does not prolong descent.
		if (thrustTarget > ALTHOLD_HOVER_THRUST) thrustTarget = ALTHOLD_HOVER_THRUST;
		mode = AUTO;
	}
	// Follow actual yaw each frame to prevent yaw integral buildup during descent.
	float currentYaw = attitude.getYaw();
	attitudeTarget = Quaternion::fromEuler(Vector(0, 0, currentYaw));

	thrustTarget -= dt / descendTime;
	if (thrustTarget < 0) {
		thrustTarget = 0;
		armed = false;
	}
}

// Allow pilot to interrupt automatic flight
void autoFailsafe() {
	static float roll, pitch, yaw, throttle;
	
	// The control variables already include SBUS, MAVLink and web input.
	if (roll != controlRoll || pitch != controlPitch || yaw != controlYaw || abs(throttle - controlThrottle) > 0.05) {
		if (mode == AUTO) mode = STAB;
	}
	
	roll = controlRoll;
	pitch = controlPitch;
	yaw = controlYaw;
	throttle = controlThrottle;
}

#if WEB_RC_ENABLED
// Web remote signal loss
void webRCLossFailsafe() {
	if (!webRCEnabled || !useWebRC) return;
	if (!armed) return;

	// Compare milliseconds directly to avoid rounding errors of up to a second.
	if (millis() - webRCLastUpdate > WEB_RC_LOSS_TIMEOUT_MS) {
		print("Web remote signal lost. Starting descent.\n");
		descend();
		webRCEnabled = false;
		useWebRC = false;
	}
}
#endif

// Stop the motors if the aircraft stays beyond the inversion threshold for 1.5 seconds.
void invertedFailsafe() {
	if (!armed) {
		isInverted = false;
		return;
	}

	// The body Z axis projected onto world Z is about +1 upright or -1 upside down.
	Vector worldUp = Quaternion::rotateVector(Vector(0, 0, 1), attitude);

	static float invertedStartTime = 0;
	if (worldUp.z < INVERTED_COS_THRESHOLD) {
		isInverted = true;
		if (invertedStartTime == 0) invertedStartTime = t;
		if (t - invertedStartTime > INVERTED_TIMEOUT) {
			armed = false;
			thrustTarget = 0.0f;
			invertedStartTime = 0;
			print("Drone upside down. Motors stopped.\n");
		}
	} else {
		isInverted = false;
		invertedStartTime = 0;
	}
}

// Battery voltage protection
// L1 blocks arming in control.ino and disarms at idle.
// L2 flashes the LED rapidly in flight.
// L3 triggers automatic descent in flight. SF_DESCEND_TIME sets the descent time.
void batteryFailsafe() {
	static bool l3Latched = false;
	static float lowSince = 0.0f;
	static float l3LastNotify = 0.0f;

	if (batteryVoltage < VBAT_ABSENT_THRESHOLD) return; // Ignore readings when the battery is disconnected.
	if (!armed) {
		l3Latched = false;
		lowSince = 0.0f;
		l3LastNotify = 0.0f;
		return;
	}

	// Once L3 triggers, keep descending until disarmed even if the voltage briefly recovers.
	if (l3Latched) {
		if (t - l3LastNotify >= 2.0f) {
			print("Critical battery (%.2f V). Landing continues at %.0f%% thrust.\n",
			      batteryVoltage, thrustTarget * 100.0f);
#if WEB_RC_ENABLED
			char warnBuf[64];
			snprintf(warnBuf, sizeof(warnBuf),
			         "Critical battery (%.2f V). Landing at %.0f%% thrust.",
			         batteryVoltage, thrustTarget * 100.0f);
			setWebRCWarn(warnBuf);
#endif
			l3LastNotify = t;
		}
		descend();
		return;
	}

	bool flying = thrustTarget >= BATTERY_FLYING_THRUST_MIN;

	bool actionCondition = false;
	bool criticalAction = false;

	// In flight, L3 starts descent. L2 only triggers the LED warning.
	if (flying) {
		if (batteryVoltage < VBAT_CRITICAL_THRESHOLD) {
			actionCondition = true;
			criticalAction = true;
		}
	} else {
		// At armed idle, L1 disarms the aircraft.
		if (batteryVoltage < VBAT_WARN_THRESHOLD) {
			actionCondition = true;
		}
	}

	if (!actionCondition) {
		lowSince = 0.0f;
		return;
	}

	if (lowSince == 0.0f) lowSince = t;
	if (t - lowSince < BATTERY_ACTION_DEBOUNCE_TIME) return;

	lowSince = 0.0f;
	if (criticalAction) {
		l3Latched = true;
		l3LastNotify = 0.0f;
		print("Critical battery (%.2f V). Starting automatic landing.\n", batteryVoltage);
#if WEB_RC_ENABLED
		char warnBuf[64];
		snprintf(warnBuf, sizeof(warnBuf),
		         "Critical battery (%.2f V). Starting automatic landing.",
		         batteryVoltage);
		setWebRCWarn(warnBuf);
#endif
		descend();
		return;
	}

	armed = false;
	thrustTarget = 0.0f;
	print("Low battery (%.2f V). Disarming automatically.\n", batteryVoltage);
#if WEB_RC_ENABLED
	char warnBuf[64];
	snprintf(warnBuf, sizeof(warnBuf), "Low battery (%.2f V). Motors disarmed.", batteryVoltage);
	setWebRCWarn(warnBuf);
#endif
}
