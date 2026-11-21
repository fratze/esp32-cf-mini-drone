// control.h
#ifndef CONTROL_H
#define CONTROL_H

// Flight modes
enum FlightMode {
    MODE_MANUAL = 0,
    MODE_ACRO = 1,
    MODE_STAB = 2,
    MODE_ALTHOLD = 3,  // Barometric altitude hold. Throttle controls climb or descent rate.
    MODE_AUTO = 4
};

// Controller state
struct ControlState {
    int mode;
    bool armed;
    float thrustTarget;
    float rollTarget;
    float pitchTarget;
    float yawTarget;
};

// Function declarations
void control();
void interpretControls();
void interpretWebRC();
void combineInputs();
void controlAttitude();
void controlRates();
void controlTorque();

// Helper functions
const char* getModeName(int mode);

// External functions
bool isUsingWebRC();
void setWebRCWarn(const char* msg);

#endif // CONTROL_H
