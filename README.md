# ESP32-H2 Zigbee Environmental Monitor

Indoor environmental monitor built on the ESP32-H2, running as a Zigbee end
device (not a router) to keep power draw low. Reports temperature, humidity,
barometric pressure, ambient light, and proximity over Zigbee, with an
SSD1306 OLED that wakes on a VCNL4010 proximity interrupt to show live
readings, then powers off after a timeout.

## Hardware

| Component | Part | Role |
|---|---|---|
| MCU | ESP32-H2 | Zigbee end device (RCP-free, single chip) |
| Environmental sensor | Bosch BME280 | Temperature, humidity, pressure (I2C) |
| Proximity/light sensor | Vishay VCNL4010 | Ambient light + proximity, interrupt-driven wake (I2C) |
| Display | SSD1306 | 128x64 OLED, shows live readings on wake, sleeps after timeout (I2C, official esp-idf lcd driver) |

Current build stage: dev board + breakout modules (perfboard/jumper wiring).
Custom PCB may follow later but is out of scope for now.

Power target: battery or mains powered end device. The ESP32-H2 will not used as a
Zigbee router in this project.

## Software Stack

- **ESP-IDF**: v6.1
- **Zigbee**: `esp-zigbee-lib` v2.x (end device role only)
- **BME280**: custom driver (this repo)
- **VCNL4010**: custom driver (this repo), interrupt-driven proximity wake

## Configuration

App-specific settings (I2C pins, sensor intervals, display timeout, Zigbee
identity strings) live in `firmware/main/Kconfig.projbuild` and are set via: ```menuconfig```
under **Application Configuration**. Zigbee stack/channel settings are
configured separately under the `esp-zigbee-lib` menu.

### Dependencies

Zigbee stack dependency is declared in `firmware/main/idf_component.yml`
and resolved automatically by the ESP-IDF Component Manager on build:
```idf.py build```.
No manual cloning of `esp-zigbee-lib` is required.

## Repository Layout

```
.
└── firmware/                     # all ESP-IDF firmware code
    ├── main/
    │   ├── Kconfig.projbuild     # project settings
    │   ├── idf_component.yml     # project dependencies
    ├── components/
    │   ├── bme280/               # custom BME280 driver
    │   └── vcnl4010/             # custom VCNL4010 driver
    ├── sdkconfig
    └── CMakeLists.txt
```

## Display Behavior

1. VCNL4010 proximity interrupt fires (e.g. hand wave / presence near sensor).
2. MCU wakes, reads BME280 + VCNL4010, updates SSD1306 with live values.
3. Display stays on for a set timeout, then powers off to save
   power until the next interrupt.

## Zigbee Network Compatibility

Target: broad Zigbee 3.0 coordinator compatibility as an end device.
Primary validation targets:
- Home Assistant + ZHA
- Home Assistant + Zigbee2MQTT

Other coordinators (deCONZ, SmartThings, etc.) should work if they follow
the Zigbee 3.0 spec for end device joining and standard/custom clusters, but
are not actively tested yet.

## Status

Early development. See `TIMELINE.md` for planned build phases.

## License

See [LICENSE](LICENSE).
