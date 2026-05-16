/*
 * ecu_state_machine.c
 * Core FSM logic for the engine-ecu-simulator.
 *
 * Design pattern: table-less explicit switch/case FSM.
 * This mirrors how Bosch's AUTOSAR-compliant ECU software is typically
 * structured — each state owns its transition logic, making it easy to
 * audit and trace during software validation (ISO 26262 / ASPICE).
 */

#include <stdio.h>
#include <string.h>
#include "ecu.h"

/* ------------------------------------------------------------------ */
/* Internal helpers                                                     */
/* ------------------------------------------------------------------ */

static void set_state(ecu_context_t *ctx, ecu_state_t new_state)
{
    printf("[ECU] State: %-10s -> %s\n",
           ecu_state_name(ctx->state),
           ecu_state_name(new_state));
    ctx->state = new_state;
}

static void raise_fault(ecu_context_t *ctx, ecu_fault_code_t code)
{
    ctx->active_faults |= code;
    printf("[ECU] FAULT raised: 0x%04X  (active mask: 0x%04X)\n",
           code, ctx->active_faults);
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

void ecu_init(ecu_context_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->state         = ECU_STATE_OFF;
    ctx->active_faults = ECU_FAULT_NONE;
    printf("[ECU] Initialised. State: OFF\n");
}

void ecu_update_sensors(ecu_context_t *ctx, const ecu_sensors_t *sensors)
{
    ctx->sensors = *sensors;
}

ecu_state_t ecu_process_event(ecu_context_t *ctx, ecu_event_t event)
{
    switch (ctx->state) {

    /* ---- OFF ---- */
    case ECU_STATE_OFF:
        if (event == ECU_EVENT_IGNITION_ON) {
            ctx->crank_ticks = 0;
            set_state(ctx, ECU_STATE_CRANKING);
        }
        break;

    /* ---- CRANKING ---- */
    case ECU_STATE_CRANKING:
        if (event == ECU_EVENT_RPM_STABLE) {
            /*
             * RPM reached idle band.
             * If throttle is already pressed (e.g. cold start blip),
             * jump straight to RUNNING; otherwise enter IDLE.
             */
            if (ctx->sensors.throttle_pos > THROTTLE_IDLE_THRESH) {
                set_state(ctx, ECU_STATE_RUNNING);
            } else {
                set_state(ctx, ECU_STATE_IDLE);
            }
        } else if (event == ECU_EVENT_IGNITION_OFF) {
            set_state(ctx, ECU_STATE_OFF);
        } else if (event == ECU_EVENT_SENSOR_ERROR) {
            raise_fault(ctx, ECU_FAULT_SENSOR);
            set_state(ctx, ECU_STATE_FAULT);
        }
        break;

    /* ---- IDLE ---- */
    case ECU_STATE_IDLE:
        if (event == ECU_EVENT_ACCEL) {
            set_state(ctx, ECU_STATE_RUNNING);
        } else if (event == ECU_EVENT_IGNITION_OFF) {
            set_state(ctx, ECU_STATE_OFF);
        } else if (event == ECU_EVENT_SENSOR_ERROR) {
            raise_fault(ctx, ECU_FAULT_SENSOR);
            set_state(ctx, ECU_STATE_FAULT);
        } else if (event == ECU_EVENT_OVERTEMP) {
            raise_fault(ctx, ECU_FAULT_OVERTEMP);
            set_state(ctx, ECU_STATE_FAULT);
        }
        break;

    /* ---- RUNNING ---- */
    case ECU_STATE_RUNNING:
        if (event == ECU_EVENT_DECEL) {
            set_state(ctx, ECU_STATE_IDLE);
        } else if (event == ECU_EVENT_IGNITION_OFF) {
            set_state(ctx, ECU_STATE_OFF);
        } else if (event == ECU_EVENT_OVERTEMP) {
            raise_fault(ctx, ECU_FAULT_OVERTEMP);
            set_state(ctx, ECU_STATE_FAULT);
        } else if (event == ECU_EVENT_SENSOR_ERROR) {
            raise_fault(ctx, ECU_FAULT_SENSOR);
            set_state(ctx, ECU_STATE_FAULT);
        }
        break;

    /* ---- FAULT ---- */
    case ECU_STATE_FAULT:
        /*
         * In FAULT state the ECU waits for a technician reset or
         * a power-cycle.  No other events are accepted — this is
         * intentional: a faulted ECU must not silently self-recover.
         */
        if (event == ECU_EVENT_RESET) {
            ctx->active_faults = ECU_FAULT_NONE;
            set_state(ctx, ECU_STATE_OFF);
        } else {
            printf("[ECU] Event ignored in FAULT state (fault must be cleared first)\n");
        }
        break;

    default:
        printf("[ECU] Unknown state %d — resetting to OFF\n", ctx->state);
        ctx->state = ECU_STATE_OFF;
        break;
    }

    return ctx->state;
}

void ecu_tick(ecu_context_t *ctx)
{
    ctx->tick_count++;

    /* --- Automatic sensor-driven fault detection --- */

    /* Overtemperature guard (active in IDLE and RUNNING) */
    if ((ctx->state == ECU_STATE_IDLE || ctx->state == ECU_STATE_RUNNING)
        && ctx->sensors.coolant_temp > COOLANT_TEMP_MAX)
    {
        ecu_process_event(ctx, ECU_EVENT_OVERTEMP);
        return;
    }

    /* Sensor health check */
    if ((ctx->state == ECU_STATE_IDLE || ctx->state == ECU_STATE_RUNNING
         || ctx->state == ECU_STATE_CRANKING)
        && (!ctx->sensors.maf_ok || !ctx->sensors.o2_ok))
    {
        ecu_process_event(ctx, ECU_EVENT_SENSOR_ERROR);
        return;
    }

    /* Crank timeout: engine failed to start within RPM_CRANK_TIMEOUT ticks */
    if (ctx->state == ECU_STATE_CRANKING) {
        ctx->crank_ticks++;
        if (ctx->crank_ticks >= RPM_CRANK_TIMEOUT) {
            printf("[ECU] Crank timeout after %u ticks\n", ctx->crank_ticks);
            raise_fault(ctx, ECU_FAULT_RPM_TIMEOUT);
            set_state(ctx, ECU_STATE_FAULT);
            return;
        }

        /* Auto-detect idle RPM during cranking */
        if (ctx->sensors.rpm >= RPM_IDLE_MIN
            && ctx->sensors.rpm <= RPM_IDLE_MAX)
        {
            ecu_process_event(ctx, ECU_EVENT_RPM_STABLE);
            return;
        }
    }

    /* Throttle-driven IDLE <-> RUNNING transitions */
    if (ctx->state == ECU_STATE_IDLE
        && ctx->sensors.throttle_pos > THROTTLE_IDLE_THRESH)
    {
        ecu_process_event(ctx, ECU_EVENT_ACCEL);
    } else if (ctx->state == ECU_STATE_RUNNING
               && ctx->sensors.throttle_pos <= THROTTLE_IDLE_THRESH)
    {
        ecu_process_event(ctx, ECU_EVENT_DECEL);
    }
}

/* ------------------------------------------------------------------ */
/* Utility                                                              */
/* ------------------------------------------------------------------ */

const char *ecu_state_name(ecu_state_t state)
{
    switch (state) {
        case ECU_STATE_OFF:      return "OFF";
        case ECU_STATE_CRANKING: return "CRANKING";
        case ECU_STATE_IDLE:     return "IDLE";
        case ECU_STATE_RUNNING:  return "RUNNING";
        case ECU_STATE_FAULT:    return "FAULT";
        default:                 return "UNKNOWN";
    }
}

void ecu_print_status(const ecu_context_t *ctx)
{
    printf("--- ECU Status (tick %u) ---\n", ctx->tick_count);
    printf("  State        : %s\n",   ecu_state_name(ctx->state));
    printf("  Faults       : 0x%04X\n", ctx->active_faults);
    printf("  RPM          : %.0f\n", ctx->sensors.rpm);
    printf("  Coolant temp : %.1f C\n", ctx->sensors.coolant_temp);
    printf("  Throttle     : %.0f%%\n", ctx->sensors.throttle_pos * 100.0f);
    printf("  MAF OK       : %s\n",   ctx->sensors.maf_ok ? "yes" : "NO");
    printf("  O2 OK        : %s\n",   ctx->sensors.o2_ok  ? "yes" : "NO");
    printf("---------------------------\n");
}