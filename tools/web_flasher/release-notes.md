ESP-Drone with browser control, {{VERSION}}. **Not tested on real hardware yet:**
remove the propellers and work through the bench checklist in
[docs/e1-mini.md](https://github.com/{{REPO}}/blob/webcontrol/docs/e1-mini.md) before the first flight.

## Install

**In the browser:** <https://technikweber.github.io/esp-drone/> (Chrome or Edge on a computer,
one click). Other ways, including Espressif's own tools:
[docs/flashing.md](https://github.com/{{REPO}}/blob/webcontrol/docs/flashing.md).

Each file is a complete image for address **0x0**:

| File | Board | Chip |
|---|---|---|
| `espdrone-e1-mini-esp32.bin` | E1 Mini ("ESP32 Drone" kit) | ESP32 |
| `espdrone-s2-drone-esp32s2.bin` | Espressif ESP32-S2-Drone | ESP32-S2 |
| `espdrone-s2-drone-esp32s3.bin` | Espressif ESP32-S2-Drone with ESP32-S3 | ESP32-S3 |

```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 write_flash 0x0 espdrone-e1-mini-esp32.bin
```

Back up the factory firmware first if you want to go back (`read_flash 0 ALL`, see the guide).

## What's in it

- Browser control over the drone's Wi-Fi with a captive portal: WebSocket, only the newest
  command counts, late commands are dropped, staged failsafe (hold, level out, descend, off).
- Battery protection: forced landing below 3.0 V, no arming below 3.4 V.
- Roll and pitch trim, stored on the drone. Control page in English and German.
- Board support for the E1 Mini (ESP32 DevKit, MPU6500 on SPI, 8520 motors) with controller
  gains from the vendor firmware.
- Based on espressif/esp-drone `master@9d5a4f8`, built with ESP-IDF 5.5.

## SHA-256

```
{{SHA256SUMS}}
```
