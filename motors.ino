// MOSFET motor output control
// For an ESC, set pwmStop, pwmMin and pwmMax to pulse widths in microseconds and reduce pwmFrequency to 400 Hz.

#include "util.h"
#include "board_config.h"

float motors[4]; // normalized motor thrusts in range [0..1]

// Motor order is rear left (0), rear right (1), front right (2), front left (3).
int motorPins[4] = BOARD_MOTOR_PINS; // RL, RR, FR, FL

int pwmFrequency = 25000; // The C3 uses a 40 MHz crystal clock for LEDC, with a 10-bit limit of 39062 Hz.
                          // ESP32 uses an 80 MHz APB clock, with a 10-bit limit of 78125 Hz.
                          // Both clocks can generate 25 kHz exactly with room to spare.
int pwmResolution = 10;
int pwmStop = 0;
int pwmMin = 0;
int pwmMax = -1; // Use -1 for direct MOSFET duty-cycle control. For an ESC, set the maximum pulse width in microseconds.

// Motors array indexes:
const int MOTOR_REAR_LEFT = 0;
const int MOTOR_REAR_RIGHT = 1;
const int MOTOR_FRONT_RIGHT = 2;
const int MOTOR_FRONT_LEFT = 3;

void setupMotors() {
	print("Setup Motors\n");

	// Detach the pins to release old LEDC channels on repeated setup, then drive them low to keep motors stopped.
	for (int i = 0; i < 4; i++) {
		ledcDetach(motorPins[i]);         // On first setup the unbound pin returns false without side effects.
		pinMode(motorPins[i], OUTPUT);
		digitalWrite(motorPins[i], LOW);
	}

	// configure pins
	for (int i = 0; i < 4; i++) {
		bool ok = ledcAttach(motorPins[i], pwmFrequency, pwmResolution);
		if (ok) {
			double actual = ledcChangeFrequency(motorPins[i], pwmFrequency, pwmResolution);
			if (actual > 0) pwmFrequency = (int)round(actual); // Keep the frequency as a double to preserve precision.
		}
		print("  motor%d pin=%d ledcAttach=%s\n", i, motorPins[i], ok ? "OK" : "FAIL");
	}

	sendMotors();
	print("Motors initialized\n");
}

int getDutyCycle(float value) {
	value = constrain(value, 0, 1);
	if (pwmMax >= 0) { // Pulse-width mode for ESCs
		float pwm = mapf(value, 0, 1, pwmMin, pwmMax);
		if (value == 0) pwm = pwmStop;
		float duty = mapf(pwm, 0, 1000000 / pwmFrequency, 0, (1 << pwmResolution) - 1);
		return round(duty);
	} else { // Duty-cycle mode for direct MOSFET drive
		return round(value * ((1 << pwmResolution) - 1));
	}
}

void sendMotors() {
	for (int i = 0; i < 4; i++) {
		ledcWrite(motorPins[i], getDutyCycle(motors[i]));
	}
}

bool motorsActive() {
	return motors[0] != 0 || motors[1] != 0 || motors[2] != 0 || motors[3] != 0;
}

void testMotor(int n) {
	print("Testing motor %d\n", n);
	motors[n] = 0.3;
	delay(50); // ESP32 may need to wait until the end of the current cycle to change duty https://github.com/espressif/arduino-esp32/issues/5306
	sendMotors();
	pause(3);
	motors[n] = 0;
	sendMotors();
	print("Done\n");
}
