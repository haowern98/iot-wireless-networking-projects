# IoT Wireless Networking Projects

Selected wireless networking and IoT systems projects from `CS4222 Wireless Networking`.

## Overview

This repository collects three embedded wireless systems projects built with Contiki-NG and the TI SensorTag CC2650 platform. The projects focus on sensor-driven state machines, low-level wireless communication with NullNet, and link-aware data transfer over intermittent wireless contact.

## Projects

- [Project 1: Sensor State Machine](project-1-sensor-state-machine/README.md)
  Sensor-driven buzzer control and event handling using motion and light thresholds.
- [Project 2: NullNet Unicast Communication](project-2-nullnet-unicast-communication/README.md)
  One-to-one packet transmission and reception with RSSI logging on Contiki-NG.
- [Project 3: Link-Aware Sensor Data Transfer](project-3-link-aware-sensor-data-transfer/README.md)
  Discovery, link evaluation, and ACK-based chunked transfer of sensor data between wireless nodes.

## Platform and Tools

- C
- Contiki-NG
- NullNet
- TI SensorTag CC2650
- Embedded sensing and radio communication

## Repository Structure

```text
iot-wireless-networking-projects/
  README.md
  project-1-sensor-state-machine/
  project-2-nullnet-unicast-communication/
  project-3-link-aware-sensor-data-transfer/
```

## Notes

- The code was originally developed for `CS4222 Wireless Networking`.
- The repository focuses on source code and rewritten documentation rather than coursework submission artifacts.
- Build instructions depend on a Contiki-NG workspace and SensorTag CC2650 target configuration.
