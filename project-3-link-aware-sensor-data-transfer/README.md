# Project 3: Link-Aware Sensor Data Transfer

Developed as part of `CS4222 Wireless Networking`.

## Overview

This project implements a more complete wireless sensing and transfer workflow between two nodes. The sender performs discovery, evaluates link quality, collects sensor readings, and transfers data in chunks with acknowledgements. The receiver reconstructs the full sensor stream and prints the recovered light and motion readings.

## What the Project Covers

- duty-cycled wireless discovery
- link-quality evaluation using RSSI or packet reception ratio (PRR)
- session-based transfer start / ready / end signaling
- chunked sensor data transfer with ACKs and retry logic
- timeout handling and recovery after contact loss

## Files

- `link_aware_sender.c`
  Implements the sensing node, including discovery, link evaluation, sensor sampling, chunked transfer, and retry / abort logic.
- `link_aware_receiver.c`
  Implements the collector node, including reply generation, transfer coordination, ACK handling, and reconstruction of the received sensor arrays.

## Key Behaviors

- sensor data is sampled into 60 readings
- readings are transmitted in chunks of 3 samples
- link quality can be evaluated using:
  - RSSI thresholding
  - PRR thresholding
- receiver output includes reconstructed `Light:` and `Motion:` sequences after successful transfer

## How to Run

1. Place both source files in a Contiki-NG example workspace.
2. Build for the `cc26x0-cc13x0` target and `sensortag/cc2650` board.
3. Flash sender and receiver binaries to separate SensorTag nodes.
4. Open serial output on both devices to observe discovery, transfer status, ACK exchange, and reconstructed sensor data.

## Notes

- The sender currently defaults to RSSI-based link evaluation through `LINK_METRIC_MODE`.
- Assignment PDFs, reports, and statement-of-work files are intentionally excluded.
- The renamed filenames in this repository are documentation-friendly versions of the original submission files.
