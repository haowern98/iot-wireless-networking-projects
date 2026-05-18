# Project 2: NullNet Unicast Communication

Developed as part of `CS4222 Wireless Networking`.

## Overview

This project demonstrates one-to-one communication between wireless nodes using Contiki-NG and NullNet. The programs send sequential packet counters, log RSSI on reception, and track how many packets have been received over time.

## What the Project Covers

- NullNet packet transmission and reception
- point-to-point communication using link-layer addresses
- RSSI logging through packet buffer attributes
- periodic packet sending with event timers

## Files

- `nullnet_transmitter.c`
  Sends sequential packets at a fixed interval and logs received replies with RSSI information.
- `nullnet_receiver.c`
  Starts in receive mode, then switches to transmit mode after a timed interval to support a bidirectional experiment.

## Platform

- Contiki-NG
- NullNet
- TI SensorTag CC2650

## How to Run

1. Place the source files in a Contiki-NG example workspace.
2. Configure the destination link-layer address in each file for the peer node.
3. Build for the `cc26x0-cc13x0` target and `sensortag/cc2650` board.
4. Flash the programs to two SensorTag nodes.
5. Open serial output to observe transmitted packet counters, received packets, and RSSI values.

## Notes

- The code sends packets at a quarter-second interval.
- The current implementation stops after 240 packets in each sending phase.
- Submission PDFs, reports, and statement-of-work files are not included.
