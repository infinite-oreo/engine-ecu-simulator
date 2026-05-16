/*
 * main.c
 * Simulation driver — runs a scripted "drive cycle" through the ECU FSM.
 *
 * Outputs a CSV log to stdout that can be piped into tools/visualize.py.
 *
 * Usage:
 *   gcc -Wall -Wextra -I include src/ecu_state_machine.c src/main.c -o ecu_sim
 *   ./ecu_sim            # human-readable log
 *   ./ecu_sim --csv      # CSV output for visualizer
 */

#include <stdio.h>
#include <string.h>
#include "ecu.h"

/* Tick period in milliseconds (simulate 10 ms ECU cycle) */
#define TICK_MS 10

/* ------------------------------------------------------------------ */
/* Scenario helpers                                                     */
/* ------------------------------------------------------------------ */

static int g_csv_mode = 0;

/* Advance simulation by n ticks with fixed sensor values */
static void run_ticks(ecu_context_t *ctx, uint32_t n,
                      float rpm, float coolant, float throttle,
                      uint8_t maf_ok, uint8_t o2_ok)
{
    ecu_sensors_t s = {rpm, coolant, throttle, maf_ok, o2_ok};
    ecu_update_sensors(ctx, &s);

    for (uint32_t i = 0; i < n; i++) {
        ecu_tick(ctx);
        if (g_csv_mode) {
            printf("%u,%s,%.0f,%.1f,%.0f,%d,%d,0x%04X\n",
                   ctx->tick_count * TICK_MS,
                   ecu_state_name(ctx->state),
                   ctx->sensors.rpm,
                   ctx->sensors.coolant_temp,
                   ctx->sensors.throttle_pos * 100.0f,
                   ctx->sensors.maf_ok,
                   ctx->sensors.o2_ok,
                   ctx->active_faults);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Main                                                                 */
/* ------------------------------------------------------------------ */

int main(int argc, char *argv[])
{
    if (argc > 1 && strcmp(argv[1], "--csv") == 0) {
        g_csv_mode = 1;
        /* Print CSV header */
        printf("time_ms,state,rpm,coolant_c,throttle_pct,maf_ok,o2_ok,faults\n");
    }

    ecu_context_t ecu;
    ecu_init(&ecu);

    if (!g_csv_mode) printf("\n=== Phase 1: Cold start ===\n");
    /* Fire ignition — ECU enters CRANKING */
    ecu_process_event(&ecu, ECU_EVENT_IGNITION_ON);

    /* Simulate starter motor: RPM ramps up over 20 ticks */
    run_ticks(&ecu, 10, 200.0f, 20.0f, 0.0f, 1, 1);  /* low RPM, cranking */
    run_ticks(&ecu, 10, 800.0f, 22.0f, 0.0f, 1, 1);  /* RPM hits idle band -> IDLE */

    if (!g_csv_mode) printf("\n=== Phase 2: Warm-up at idle ===\n");
    run_ticks(&ecu, 30, 850.0f, 45.0f, 0.0f, 1, 1);  /* idle, temperature rising */

    if (!g_csv_mode) printf("\n=== Phase 3: Driver accelerates ===\n");
    run_ticks(&ecu, 5,  850.0f, 60.0f, 0.0f, 1, 1);  /* still idle */
    run_ticks(&ecu, 40, 3500.0f, 90.0f, 0.6f, 1, 1); /* throttle 60% -> RUNNING */

    if (!g_csv_mode) printf("\n=== Phase 4: Driver lifts off ===\n");
    run_ticks(&ecu, 20, 1100.0f, 92.0f, 0.0f, 1, 1); /* throttle 0 -> IDLE */

    if (!g_csv_mode) printf("\n=== Phase 5: Over-temperature fault ===\n");
    run_ticks(&ecu, 10, 1100.0f, 115.0f, 0.0f, 1, 1); /* coolant > 110°C -> FAULT */

    if (!g_csv_mode) {
        ecu_print_status(&ecu);
        printf("\n=== Phase 6: Technician resets fault ===\n");
    }
    ecu_process_event(&ecu, ECU_EVENT_RESET);

    if (!g_csv_mode) {
        ecu_print_status(&ecu);
        printf("\nSimulation complete. Run with --csv and pipe to tools/visualize.py for graphs.\n");
    }

    return 0;
}