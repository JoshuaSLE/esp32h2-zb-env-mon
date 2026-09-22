# Project Timeline (Phase Order, No Fixed Dates)

## Phase 0 — Environment Setup

* Install/verify ESP-IDF v6.1 toolchain for ESP32-H2
* Confirm `esp-zigbee-lib` ^2 pulls in correctly as a managed component
* Wire dev board + BME280 + VCNL4010 + SSD1306 breakouts on perfboard/jumpers
* Verify I2C bus addressing/conflicts across all three sensors

## Phase 1 — Sensor Bring-Up (no Zigbee yet)

* BME280 custom driver: init, calibration read, temp/humidity/pressure read
* VCNL4010 custom driver: init, proximity + ambient light read, interrupt
configuration and ISR/GPIO wake handling
* SSD1306 via official LCD driver: init, basic text/number rendering
* Confirm VCNL4010 interrupt reliably triggers display wake; confirm
display timeout/power-off logic

## Phase 2 — Zigbee End Device Bring-Up

* Basic Zigbee end device join (no clusters yet) using esp-zigbee-lib v2
* Confirm end-device-only role (no routing) and joining/rejoin behavior
* Validate against Home Assistant ZHA
* Validate against Home Assistant Zigbee2MQTT

## Phase 3 — Cluster Mapping \& Reporting

* Map BME280 data to standard Zigbee clusters (temperature, humidity,
pressure) where standard clusters exist
* Implement attribute reporting intervals/thresholds
* Re-test against ZHA and Zigbee2MQTT after cluster changes

## Phase 4 — Power Optimization

* Measure current draw: idle, sensor read, display-on, Zigbee TX
* Implement sleep strategy between VCNL4010 interrupts (light/deep sleep
as appropriate for end device role)
* Re-validate Zigbee rejoin behavior after sleep/wake cycles
* Battery runtime estimation vs. mains-powered baseline

## Phase 5 — Broader Coordinator Compatibility

* Test against additional coordinators (deCONZ, SmartThings, others as
available)
* Note any coordinator-specific quirks in README

## Phase 6 — Polish \& Documentation

* Finalize component READMEs for `bme280`, `vcnl4010`, `ssd1306` drivers
* Document wiring/pinout for the perfboard build
* Add troubleshooting notes (join failures, I2C issues, sensor drift)
* Decide on and add license

## Phase 7 (Optional, Later) — Custom PCB

* Not required for current scope; revisit once perfboard build is stable
