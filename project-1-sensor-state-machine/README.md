# Project 1: Sensor State Machine

Developed as part of `CS4222 Wireless Networking`.

## Overview

This project implements two Contiki-NG sensor applications for the TI SensorTag CC2650. Both programs use light and motion readings to drive event-based state transitions and buzzer feedback.

## What the Project Covers

- sensor polling with Contiki event timers
- threshold-based detection for motion and light changes
- finite-state-machine control logic
- buzzer activation and timed wait cycles

## Files

- `light_motion_trigger.c`
  Implements a three-state workflow (`IDLE`, `BUZZ`, `WAIT`) that reacts to light or motion deltas and triggers repeated buzzer cycles.
- `sensor_state_machine.c`
  Implements a more complex four-state workflow (`IDLE`, `INTERIM`, `BUZZ`, `WAIT`) with separate motion and light detection phases and interruption handling.

## How to Run

1. Place the source file in a Contiki-NG example workspace.
2. Use a build configuration targeting `cc26x0-cc13x0` with the `sensortag/cc2650` board.
3. Flash the compiled binary to the SensorTag.
4. Observe serial output while changing the surrounding light or moving the device.

## Notes

- Assignment PDFs and statement-of-work files are intentionally excluded.
- The source files were renamed for clarity in this repository.
