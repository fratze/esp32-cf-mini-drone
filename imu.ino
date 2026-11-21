// Gyroscope and accelerometer settings

#include <SPI.h>
#include "drivers/MPU9250.h"
#include "vector.h"
#include "lpf.h"
#include "util.h"
#include "board_config.h"

MPU9250 imu(SPI, BOARD_SPI_CS);

bool imuOK = false; // True after successful IMU setup. On failure, block arming and skip the wait in readIMU().

// IMU mounting angles in radians. The default (0, 0, -PI/2) matches this PCB.
// The chip faces up, with X pointing right and Y forward. Map readings to Vector(data.y, -data.x, data.z).
// For a different mount, change this default and run preset to reset stored parameters.
Vector imuRotation(0, 0, -PI / 2);

Vector accBias;
Vector accScale(1, 1, 1);
Vector gyroBias;
LowPassFilter<Vector> gyroBiasFilter(0.001); // Low-pass filter for estimating gyro bias

void setupIMU() {
	print("Setup IMU\n");
	SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI, BOARD_SPI_CS);
	if (!imu.begin()) {
		print("IMU setup failed. Check the IMU connections.\n");
		return; // Skip IMU configuration and timer setup. Leave imuOK false.
	}
	imuOK = true;
	configureIMU();
}

void configureIMU() {
	imu.setAccelRange(imu.ACCEL_RANGE_4G);
	imu.setGyroRange(imu.GYRO_RANGE_2000DPS);
	imu.setDLPF(imu.DLPF_MAX);
	imu.setRate(imu.RATE_1KHZ_APPROX);
	imu.setupInterrupt();
}

void readIMU() {
	if (!imuOK) return; // On IMU failure, keep gyro and acceleration at zero and let the main loop continue.
	imu.waitForData();
	imu.getGyro(gyro.x, gyro.y, gyro.z);
	imu.getAccel(acc.x, acc.y, acc.z);
	calibrateGyroOnce();
	// apply scale and bias
	acc = (acc - accBias) / accScale;
	gyro = gyro - gyroBias;
	// rotate to body frame using imuRotation Euler angles
	// Cache the mounting quaternion. imuRotation changes only when a parameter changes.
	// Each frame needs three comparisons instead of six sine and cosine calls.
	// readIMU() refreshes the cache on the next frame after a setting changes (about 1 ms).
	static Quaternion _imuRotQuat;
	static float _cachedRotX = NAN, _cachedRotY = NAN, _cachedRotZ = NAN;
	if (imuRotation.x != _cachedRotX || imuRotation.y != _cachedRotY || imuRotation.z != _cachedRotZ) {
		_imuRotQuat = Quaternion::fromEuler(imuRotation).inversed();
		_cachedRotX = imuRotation.x;
		_cachedRotY = imuRotation.y;
		_cachedRotZ = imuRotation.z;
	}
	acc  = Quaternion::rotateVector(acc,  _imuRotQuat);
	gyro = Quaternion::rotateVector(gyro, _imuRotQuat);
}

void calibrateGyroOnce() {
	static Delay landedDelay(2);
	if (!landedDelay.update(landed)) return; // calibrate only if definitely stationary

	gyroBias = gyroBiasFilter.update(gyro);
}

void calibrateAccel() {
	print("Calibrating the accelerometer\n");
	imu.setAccelRange(imu.ACCEL_RANGE_2G); // the most sensitive mode

	print("1/6 Place the drone upright on a level surface, nose forward. Hold still for 8 seconds.\n");
	pause(8);
	calibrateAccelOnce();
	print("Level position recorded.\n");
	pause(1);
	print("2/6 Point the nose straight up and rest the tail on the surface. Hold still for 8 seconds.\n");
	pause(8);
	calibrateAccelOnce();
	print("Nose-up position recorded.\n");
	pause(1);
	print("3/6 Point the nose straight down onto the surface, tail up. Hold still for 8 seconds.\n");
	pause(8);
	calibrateAccelOnce();
	print("Nose-down position recorded.\n");
	pause(1);
	print("4/6 Rest the drone on its right side, left arms up and nose forward. Hold still for 8 seconds.\n");
	pause(8);
	calibrateAccelOnce();
	print("Right-side position recorded.\n");
	pause(1);
	print("5/6 Rest the drone on its left side, right arms up and nose forward. Hold still for 8 seconds.\n");
	pause(8);
	calibrateAccelOnce();
	print("Left-side position recorded.\n");
	pause(1);
	print("6/6 Lay the drone upside down on a level surface, nose forward. Hold still for 8 seconds.\n");
	pause(8);
	calibrateAccelOnce();

	printIMUCalibration();
	print("Calibration complete. Turn the drone upright and run ps to check the result.\n");
	configureIMU();
}

void calibrateAccelOnce() {
	const int samples = 1000;
	static Vector accMax(-INFINITY, -INFINITY, -INFINITY);
	static Vector accMin(INFINITY, INFINITY, INFINITY);

	// Compute the average of the accelerometer readings
	acc = Vector(0, 0, 0);
	for (int i = 0; i < samples; i++) {
		imu.waitForData();
		Vector sample;
		imu.getAccel(sample.x, sample.y, sample.z);
		acc = acc + sample;

	#ifdef CONFIG_IDF_TARGET_ESP32C3
		// Yield for 1 ms every 10 frames so sampling on the single-core C3 does not starve Wi-Fi and cause missed beacons.
		if (i % 10 == 0) delay(1);
	#endif
	
	}
	acc = acc / samples;

	// Update the maximum and minimum values
	if (acc.x > accMax.x) accMax.x = acc.x;
	if (acc.y > accMax.y) accMax.y = acc.y;
	if (acc.z > accMax.z) accMax.z = acc.z;
	if (acc.x < accMin.x) accMin.x = acc.x;
	if (acc.y < accMin.y) accMin.y = acc.y;
	if (acc.z < accMin.z) accMin.z = acc.z;
	// Compute scale and bias
	accScale = (accMax - accMin) / 2 / ONE_G;
	accBias = (accMax + accMin) / 2;
}

void printIMUCalibration() {
	print("gyro bias: %f %f %f\n", gyroBias.x, gyroBias.y, gyroBias.z);
	print("accel bias: %f %f %f\n", accBias.x, accBias.y, accBias.z);
	print("accel scale: %f %f %f\n", accScale.x, accScale.y, accScale.z);
}

void printIMUInfo() {
	imu.status() ? print("status: ERROR %d\n", imu.status()) : print("status: OK\n");
	print("model: %s\n", imu.getModel());
	print("who am I: 0x%02X\n", imu.whoAmI());
	print("rate: %.0f\n", loopRate);
	print("gyro: %f %f %f\n", gyro.x, gyro.y, gyro.z);
	print("acc: %f %f %f\n", acc.x, acc.y, acc.z);
	imu.waitForData();
	Vector rawGyro, rawAcc;
	imu.getGyro(rawGyro.x, rawGyro.y, rawGyro.z);
	imu.getAccel(rawAcc.x, rawAcc.y, rawAcc.z);
	print("raw gyro: %f %f %f\n", rawGyro.x, rawGyro.y, rawGyro.z);
	print("raw acc: %f %f %f\n", rawAcc.x, rawAcc.y, rawAcc.z);
}
