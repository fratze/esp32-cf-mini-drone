// Battery voltage monitor

#include "board_config.h"

#define VBAT_ADC_PIN       BOARD_VBAT_ADC_PIN     // Battery voltage input pin
#define VBAT_ADC_SAMPLES   BOARD_VBAT_ADC_SAMPLES  // Use 8 ADC samples on C3 to reduce delays, or 16 on ESP32 and S3 to reduce noise.
#define VBAT_DIVIDER       (43.0f / 33.0f)  // Voltage divider with 10 kΩ above and 33 kΩ below. Vbat = Vadc × 43/33.
#define VBAT_WARN_THRESHOLD     3.5f  // L1 warning threshold at idle or with no load (V)
#define VBAT_LOW_THRESHOLD      3.3f  // L2 low-battery threshold in flight (V)
#define VBAT_CRITICAL_THRESHOLD 3.0f  // L3 critical threshold for automatic landing in flight (V)
#define VBAT_ABSENT_THRESHOLD   0.5f  // Below this voltage, assume the battery is disconnected and skip warnings.

#define BATTERY_FLYING_THRUST_MIN    0.15f  // At or above this thrust, use the in-flight thresholds instead of L1.
#define BATTERY_ACTION_DEBOUNCE_TIME  0.9f  // Seconds the voltage must stay low before protection takes effect

extern float t;
float batteryVoltage = 0.0f;  // Latest voltage from updateBatteryVoltage()

void updateBatteryVoltage() {
	static float lastCheck = 0;
	if (t - lastCheck < 0.5f) return;  // Sample every 0.5 seconds.
	lastCheck = t;
	batteryVoltage = readBatteryVoltage();
}

float readBatteryVoltage() {
	long sum = 0;
	for (int i = 0; i < VBAT_ADC_SAMPLES; i++) sum += analogReadMilliVolts(VBAT_ADC_PIN);
	return (sum / (float)VBAT_ADC_SAMPLES / 1000.0f) * VBAT_DIVIDER;
}
