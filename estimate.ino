// Estimate orientation from the gyroscope and accelerometer.

#include "quaternion.h"
#include "vector.h"
#include "lpf.h"
#include "util.h"

float accWeight = 0.003;

// ============== Proportional leveling correction ==============
float levelWeight = 0;  // Leveling gain. Disabled because the correction can worsen drift when gyro temperature drift and mechanical imbalance look alike.
float levelMaxTilt = radians(30); // rad, level correction fades out at this tilt angle (matches TILT_MAX)
// Stick threshold for leveling correction
// Use the larger of the absolute roll and pitch stick positions (range 0 to 1).
// Reduce applyLevel correction linearly to zero at the threshold 
// to avoid drift from interaction with the PID integral.
// Adjusting the threshold
// A lower value stops correction with less stick movement and 
// reduces drift correction in hover.
// A higher value lets applyLevel interfere with stick input for longer.
// Suggested range is 0.2 to 0.4. At the default of 0.2, correction stops at 20% stick travel.
// Set it in the console with p EST_LVL_GATE_THR <value>.
float levelGateThreshold = 0.2f; // Stick threshold (EST_LVL_GATE_THR)

// ============== Mahony-style integral leveling correction ==============
// levelBiasGain integrates gravity direction error into the estimated gyro bias.
// applyGyro() subtracts the integral estimate from the gyro readings.
// This avoids feeding the correction directly into the attitude PID 
// and exciting the inner integral loop.
// Higher gain converges faster but can overshoot. Settling takes about 30 seconds.
// Parameter name: EST_LVL_BIAS_GAIN
float levelBiasGain = 0;  // Mahony integral gain. Disabled because mechanical imbalance can be mistaken for gyro temperature drift.
Vector levelGyroBias(0, 0, 0); // Gyro bias estimated by applyLevel() in rad/s

extern float controlRoll, controlPitch; // Pilot stick input from the main sketch
LowPassFilter<Vector> ratesFilter(0.2); // cutoff frequency ~ 40 Hz

void estimate() {
	applyGyro();
	applyAcc();
	applyLevel();
}

void applyGyro() {
	// Subtract the estimated gyro bias before filtering and integrating.
	// This applies long-term correction through the bias estimate, 
	// outside the PID error input.
	rates = ratesFilter.update(gyro - levelGyroBias);

	// apply rates to attitude
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
	// test should we apply accelerometer gravity correction
	float accNorm = acc.norm();
	landed = !motorsActive() && abs(accNorm - ONE_G) < ONE_G * 0.1f;

	if (!landed) return;

	// calculate accelerometer correction
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector correction = Vector::rotationVectorBetween(acc, up) * accWeight;

	// apply correction
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

// Mahony-style leveling with proportional attitude correction 
// and integral gyro bias correction
// How the two terms work
// The P term (levelWeight) corrects attitude quickly.
//   Its strength was halved to reduce sudden changes.
// The I term (levelBiasGain) integrates gravity direction error into levelGyroBias.
// applyGyro() subtracts this bias from the readings before the PID sees them.
// This avoids oscillations from interaction with the inner integral loop.
void applyLevel() {
	if (landed) {
		levelGyroBias = Vector(0, 0, 0); // Clear the bias after landing and learn it again on the next flight.
		return;
	}

	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	float tilt = acos(constrain(up.z, -1.0f, 1.0f));

	// Skip correction near level because vibration can make the cross product point in a random direction.
	if (tilt < radians(0.1f)) return;

	// Reduce the P weight as tilt approaches levelMaxTilt.
	float dynamicWeight = levelWeight * constrain(1.0f - tilt / levelMaxTilt, 0.0f, 1.0f);

	// ---- Adjust correction strength based on stick position. ----
	float stickDeflection = max(abs(controlRoll), abs(controlPitch));
	float stickGate = constrain(1.0f - stickDeflection / levelGateThreshold, 0.0f, 1.0f);
	dynamicWeight *= stickGate;
	// ------------------------------------

	// Gravity direction error is the rotation between the body and world Z axes.
	Vector error = Vector::rotationVectorBetween(Vector(0, 0, 1), up);

	// A small P term corrects attitude directly for a quick response to drift.
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(error * dynamicWeight));

	// The I term accumulates estimated gyro bias.
	// Pause integration when stickGate is zero so pilot input does not build up a false bias.
	levelGyroBias += error * (levelBiasGain * stickGate);

	// Limit the correction to the equivalent of 3°/s to bound the bias estimate.
	float biasNorm = levelGyroBias.norm();
	const float biasLimit = radians(3.0f);
	if (biasNorm > biasLimit) levelGyroBias = levelGyroBias * (biasLimit / biasNorm);
}
