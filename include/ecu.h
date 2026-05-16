#ifndef ECU_H
#define ECU_H

#include <stdint.h>

/* ============================================================
 * ECU State Machine — engine-ecu-simulator
 * Simplified model of a Bosch-style Engine Control Unit
 * ============================================================ */

/* ---------- State definitions ---------- */
typedef enum {
    ECU_STATE_OFF      = 0,  /* Engine not running, all systems dormant */
    ECU_STATE_CRANKING = 1,  /* Starter motor engaged, waiting for ignition */
    ECU_STATE_IDLE     = 2,  /* Engine running at idle RPM (no throttle input) */
    ECU_STATE_RUNNING  = 3,  /* Engine under load (throttle > threshold) */
    ECU_STATE_FAULT    = 4   /* Fault detected — safe shutdown required */
} ecu_state_t;

/* ---------- Event definitions ---------- */
/* Events are the inputs that drive state transitions */
typedef enum {
    ECU_EVENT_IGNITION_ON  = 0,  /* Driver turns ignition key / start button */
    ECU_EVENT_IGNITION_OFF = 1,  /* Driver turns off ignition */
    ECU_EVENT_RPM_STABLE   = 2,  /* RPM reached idle threshold after cranking */
    ECU_EVENT_ACCEL        = 3,  /* Throttle position > THROTTLE_IDLE_THRESHOLD */
    ECU_EVENT_DECEL        = 4,  /* Throttle returned to idle position */
    ECU_EVENT_SENSOR_ERROR = 5,  /* Critical sensor read failure (MAF, O2, etc.) */
    ECU_EVENT_OVERTEMP     = 6,  /* Coolant temperature exceeded safe limit */
    ECU_EVENT_RESET        = 7   /* Fault cleared — system can return to OFF */
} ecu_event_t;

/* ---------- Fault codes ---------- */
typedef enum {
    ECU_FAULT_NONE        = 0x0000,
    ECU_FAULT_SENSOR      = 0x0001,  /* DTC P0100 – Mass Air Flow sensor */
    ECU_FAULT_OVERTEMP    = 0x0002,  /* DTC P0217 – Engine over temperature */
    ECU_FAULT_RPM_TIMEOUT = 0x0004   /* Crank timeout — engine failed to start */
} ecu_fault_code_t;

/* ---------- Sensor data ---------- */
/* Real ECUs read these from ADC channels or CAN bus.
 * Here we model them as a simple struct updated each tick. */
typedef struct {
    float    rpm;            /* Engine speed [rev/min], 0–7000 */
    float    coolant_temp;   /* Coolant temperature [°C], normal 80–100 */
    float    throttle_pos;   /* Throttle position [0.0–1.0] */
    uint8_t  maf_ok;         /* Mass Air Flow sensor status (1 = healthy) */
    uint8_t  o2_ok;          /* Oxygen sensor status (1 = healthy) */
} ecu_sensors_t;

/* ---------- ECU context ---------- */
/* This struct holds the complete runtime state of the ECU.
 * In a real system this would live in non-volatile RAM. */
typedef struct {
    ecu_state_t      state;           /* Current FSM state */
    ecu_fault_code_t active_faults;   /* Bitmask of active DTCs */
    ecu_sensors_t    sensors;         /* Last sensor reading */
    uint32_t         crank_ticks;     /* Ticks spent in CRANKING state */
    uint32_t         tick_count;      /* Total simulation ticks elapsed */
} ecu_context_t;

/* ---------- Thresholds ---------- */
#define RPM_IDLE_MIN          600.0f   /* Minimum stable idle [RPM] */
#define RPM_IDLE_MAX         1200.0f   /* Maximum idle RPM */
#define RPM_CRANK_TIMEOUT      50      /* Ticks before crank timeout fault */
#define THROTTLE_IDLE_THRESH   0.05f   /* Throttle % considered "pressed" */
#define COOLANT_TEMP_MAX      110.0f   /* Over-temperature limit [°C] */

/* ---------- Public API ---------- */

/**
 * ecu_init — reset the ECU context to factory defaults.
 * Call once before the simulation loop.
 */
void ecu_init(ecu_context_t *ctx);

/**
 * ecu_process_event — drive the state machine with a new event.
 * Returns the new state after processing.
 *
 * This is the core FSM function.  All state transitions live here.
 */
ecu_state_t ecu_process_event(ecu_context_t *ctx, ecu_event_t event);

/**
 * ecu_tick — called every simulation cycle (e.g. 10 ms).
 * Updates sensor-derived events internally (overtemp, RPM timeout, etc.)
 * and calls ecu_process_event automatically when thresholds are crossed.
 */
void ecu_tick(ecu_context_t *ctx);

/**
 * ecu_update_sensors — inject new sensor values into the context.
 * In hardware this would be triggered by an ADC interrupt.
 */
void ecu_update_sensors(ecu_context_t *ctx, const ecu_sensors_t *sensors);

/**
 * ecu_state_name — return a human-readable state name (for logging).
 */
const char *ecu_state_name(ecu_state_t state);

/**
 * ecu_print_status — dump current ECU state to stdout (debugging).
 */
void ecu_print_status(const ecu_context_t *ctx);

#endif /* ECU_H */