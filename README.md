# engine-ecu-simulator

A lightweight Engine Control Unit (ECU) state machine simulator written in C.  
Models the core control logic of a gasoline engine — from cold start to fault detection — using a deterministic finite state machine (FSM).

> Built as a portfolio project to demonstrate embedded software skills relevant to automotive ECU development
---

## State Machine Design

The ECU is modelled as a 5-state FSM driven by sensor events:

```
                  ignition on
   [OFF] ───────────────────────► [CRANKING]
    ▲                                  │
    │                     RPM stable   │
    │              ┌───────────────────┤
    │              │                   │ throttle > 0
    │              ▼                   ▼
    │           [IDLE] ◄─────── [RUNNING]
    │              │   decel/accel     │
    │    ignition  │                   │
    │─────────off──┘                   │
    │                                  │
    │        sensor error / overtemp   │
    │         ┌────────────────────────┘
    │         ▼
    │      [FAULT]
    │         │
    └─────────┘  reset
```

| State | Description |
|---|---|
| `OFF` | Engine stopped, all systems dormant |
| `CRANKING` | Starter motor engaged, waiting for stable RPM |
| `IDLE` | Engine running at idle (~800 RPM), no throttle input |
| `RUNNING` | Engine under load, throttle position > 5% |
| `FAULT` | Critical fault detected — safe shutdown, await technician reset |

---

## Fault Codes

| Code | Hex | Trigger |
|---|---|---|
| `ECU_FAULT_SENSOR` | `0x0001` | MAF or O2 sensor failure |
| `ECU_FAULT_OVERTEMP` | `0x0002` | Coolant temperature > 110 °C |
| `ECU_FAULT_RPM_TIMEOUT` | `0x0004` | Engine failed to start within timeout |

Faults are stored as a bitmask — multiple faults can be active simultaneously.

---

## Project Structure

```
engine-ecu-simulator/
├── include/
│   └── ecu.h                 # Public API, state/event/fault definitions, thresholds
├── src/
│   ├── ecu_state_machine.c   # Core FSM logic + sensor-driven tick loop
│   └── main.c                # Scripted drive-cycle simulation (supports --csv output)
├── tools/
│   └── visualize.py          # (coming soon) RPM / temperature / state plot
├── tests/
│   └── test_state_machine.c  # (coming soon) Unit tests
└── README.md
```

---

## Build & Run

**Requirements:** GCC or Clang, any POSIX system (macOS / Linux)

```bash
# Clone
git clone https://github.com/infinite-oreo/engine-ecu-simulator.git
cd engine-ecu-simulator

# Compile
gcc -Wall -Wextra -I include src/ecu_state_machine.c src/main.c -o ecu_sim

# Run (human-readable log)
./ecu_sim

# Run (CSV output for visualizer)
./ecu_sim --csv
```

**Expected output:**

```
[ECU] Initialised. State: OFF

=== Phase 1: Cold start ===
[ECU] State: OFF        -> CRANKING
[ECU] State: CRANKING   -> IDLE

=== Phase 3: Driver accelerates ===
[ECU] State: IDLE       -> RUNNING

=== Phase 5: Over-temperature fault ===
[ECU] FAULT raised: 0x0002  (active mask: 0x0002)
[ECU] State: IDLE       -> FAULT

=== Phase 6: Technician resets fault ===
[ECU] State: FAULT      -> OFF
```

---

## Simulated Drive Cycle

The `main.c` simulation runs through 6 scripted phases:

| Phase | Description | Key event |
|---|---|---|
| 1 | Cold start | Ignition ON → RPM stabilises → IDLE |
| 2 | Warm-up at idle | Coolant temperature rises to operating range |
| 3 | Acceleration | Throttle > 5% → RUNNING |
| 4 | Deceleration | Throttle returns to 0 → IDLE |
| 5 | Over-temperature fault | Coolant > 110 °C → FAULT + DTC 0x0002 |
| 6 | Technician reset | Fault cleared → OFF |

---

## Technical Highlights

- **Deterministic FSM** — all state transitions are explicit and auditable, following patterns used in AUTOSAR-compliant ECU software
- **Sensor-driven tick loop** — `ecu_tick()` runs every 10 ms and automatically raises fault events when thresholds are crossed (ISO 26262 style)
- **Bitmask fault codes** — multiple DTCs (Diagnostic Trouble Codes) can be active simultaneously, mirroring real OBD-II fault storage
- **Zero dynamic allocation** — no `malloc`/`free`; all state lives in a single `ecu_context_t` struct, suitable for bare-metal embedded targets
- **Clean separation of interface and implementation** — `ecu.h` defines the public API; `ecu_state_machine.c` contains the implementation, making it easy to swap control logic without touching the caller
- **Conventional Commits** — commit history follows the `feat:` / `fix:` / `chore:` / `docs:` convention

---

## Roadmap

- [ ] `tools/visualize.py` — matplotlib plot of RPM, coolant temperature, and state over time
- [ ] `tests/test_state_machine.c` — unit tests for all state transitions
- [ ] `docs/design.md` — full state machine specification with timing diagrams
- [ ] Engine physical model (`engine_model.c`) — RPM dynamics, fuel injection logic

---
