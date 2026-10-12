// Global parameters
#define HZ 50
#define G 9.807
#undef DEBUG
#define SIMULATION

// Flight profile
#define HOVER_ALTITUDE_MM 1000
#define LANDING_ALTITUDE_MM 200
#define HOVER_SECONDS 10
#define THROTTLE_UP_SECONDS 1
#define ABORT_THRESHOLD_DEG 40
#define STARTUP_SECONDS 20

// Hardware parameters
#define UNPOWERED_TVC
#define UNPOWERED_ESC
#define COUNDOWN_SECONDS 10
#define COUNTDOWN_FREQUENCY 220
#define COUNTDOWN_BEEP_SECONDS 0.5
#define WIGGLE_T_MINUS 15
#define WIGGLE_ANGLE 45
#define WIGGLE_STEP 0.15
#define WIGGLE_INTERVAL 0.5

// Control parameters
#define KP 2
#define KI 0.25
#define KD 5
#define RKD 25

#define AKP 1
#define AKI 0.25
#define AKD 50
#define MIN_THROTTLE 20.0
#define MAX_THROTTLE 90.0

// BMS parameters
#define VOLTAGE_THRESHOLD_V 3.4
#define BMS_HZ 2.0
#define DISABLE_BMS

// PID
#define INTEGRAL_LIMIT 10.0f
#define INTEGRAL_RATE 1.0f
