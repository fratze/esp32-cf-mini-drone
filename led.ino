// Onboard status LED

#include "board_config.h"

#if BOARD_LED_ENABLED

#define BLINK_PERIOD      500000  // Slow blink at 1 Hz (500 ms half-period)
#define BLINK_FAST_PERIOD  62500  // Fast blink at 8 Hz (62.5 ms half-period)


// ---- External declarations ----
extern bool armed;
extern float t;
extern float controlTime;
extern float rcLossTimeout;
extern float thrustTarget;    // control.ino
extern float batteryVoltage;  // battery.ino
extern bool isInverted; // safety.ino
#if WEB_RC_ENABLED
extern bool webRCEnabled;
extern bool useWebRC;
bool isUsingWebRC();
#endif

void setupLED() {
	pinMode(BOARD_LED_PIN, OUTPUT);
	digitalWrite(BOARD_LED_PIN, BOARD_LED_INVERTED ? HIGH : LOW); // Start with the LED off.
}

void setLED(bool on) {
	static bool state = false;
	if (on == state) {
		return; // don't call digitalWrite if the state is the same
	}
	digitalWrite(BOARD_LED_PIN, (on ^ BOARD_LED_INVERTED) ? HIGH : LOW);
	state = on;
}

void blinkLED() {
	setLED(micros() / BLINK_PERIOD % 2);
}

// Select the low-battery warning threshold for the flight state.
// In flight (thrustTarget >= 0.15), use L2.
// When disarmed or idling, use L1.
bool batteryAlertActive() {
	if (batteryVoltage <= VBAT_ABSENT_THRESHOLD) return false;
	bool flying = armed && thrustTarget >= 0.15f;
	if (flying) return batteryVoltage < VBAT_LOW_THRESHOLD;   // L2 in flight
	else        return batteryVoltage < VBAT_WARN_THRESHOLD;   // L1 when disarmed or idling
}

// Check for low battery, lost control signal or an upside-down aircraft.
bool ledAlertActive() {
	// Check for an upside-down aircraft.
	if (isInverted) return true;

	// Check the radio receiver for signal loss while armed.
	if (controlTime != 0 && armed && (t - controlTime > rcLossTimeout)) return true;

#if WEB_RC_ENABLED
	// Check whether an active web remote has timed out.
	if (webRCEnabled && useWebRC && !isUsingWebRC()) return true;
#endif

	if (batteryAlertActive()) return true;

	return false;
}

// Update the LED from the main loop.
void updateLED() {
	if (!armed) {
		if (batteryAlertActive()) {
			setLED(micros() / BLINK_FAST_PERIOD % 2); // Blink fast if the battery is low before arming.
		} else {
			setLED(false); // Keep the LED off in normal standby.
		}
		return;
	}
	if (ledAlertActive()) {
		setLED(micros() / BLINK_FAST_PERIOD % 2); // Blink at 8 Hz for warnings.
	} else {
		setLED(micros() / BLINK_PERIOD % 2); // Blink at 1 Hz during normal flight.
	}
}

#else // LED support is disabled.

void setupLED() {}
void setLED(bool on) {}
void updateLED() {}

#endif // BOARD_LED_ENABLED
