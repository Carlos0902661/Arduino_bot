# Phase-1 Motor + Line-Follow + Serial Barcode Test (Archived)

## What this is
An archived Phase-1 Arduino Nano test harness for early line-following, initiator gate detection, and clocked-bar decoding over Serial.

## What it does
- Drives motors with a simple line-follow rule table using simulated L/C/R sensor input.
- Detects an initiator gate (111) with debounce, then switches to a READING state.
- Decodes clocked-bar timing from Serial BAR commands into a bit buffer.

## How to test (Serial commands)
Open the Serial Monitor at 115200 baud and send newline-terminated commands:
- `010` / `111` / etc. to simulate the L/C/R sensors.
- `S` to print status.
- `BAR <ms>` (e.g., `BAR 120`, `BAR 240`) to append barcode timing while in READING.
- `END` or `R` to return to FOLLOW (bits preserved).
- `RESET` to clear bits and return to FOLLOW.

## Status
Archived (Phase-1). This sketch is not active and has been superseded by later phases in the repository.

**Phase tag:** Phase-1
