# Flashing the firmware

Ready-made firmware is attached to every
[release](https://github.com/TechnikWeber/esp-drone/releases). Each file is a
complete image (bootloader, partition table and app in one file) that is
written at address **0x0**:

| File | Board | Chip |
|---|---|---|
| `espdrone-e1-mini-esp32.bin` | E1 Mini ("ESP32 Drone" kit), see [e1-mini.md](e1-mini.md) | ESP32 |
| `espdrone-s2-drone-esp32s2.bin` | Espressif ESP32-S2-Drone | ESP32-S2 |
| `espdrone-s2-drone-esp32s3.bin` | Espressif ESP32-S2-Drone with ESP32-S3 | ESP32-S3 |

Before the first flash:

- **Nothing has been tested on real hardware yet.** Remove the propellers and
  work through the bench checklist in [e1-mini.md](e1-mini.md).
- **Back up the factory firmware** if you want to go back:
  `esptool.py --port /dev/ttyUSB0 read_flash 0 ALL backup.bin` (details in
  [e1-mini.md](e1-mini.md#back-up-the-factory-firmware-before-the-first-flash)).
  Only esptool (way D) can read the flash; the browser tools cannot.
- Writing the full image also resets the settings stored on the drone, such as
  the trim.
- On Linux, access to the serial port needs the group `dialout`
  (`sudo usermod -aG dialout $USER`, then log out and back in). This applies to
  the browser tools as well.

## A. In the browser, one click (recommended)

<https://technikweber.github.io/esp-drone/>

1. Open the page in **Chrome or Edge on a computer** (Web Serial; phones,
   Firefox and Safari cannot flash).
2. Connect the drone's ESP32 by USB.
3. Click "Install" for your board, pick the serial port, tick "Erase device"
   for the first install and confirm. It takes about a minute.

The page picks the right image from the chip, so the E1 Mini button refuses an
ESP32-S3, for example.

## B. Espressif's web flasher (esptool-js)

Espressif's own browser tool, also Chrome or Edge on a computer.

1. Download the `.bin` for your board from the release.
2. Open <https://espressif.github.io/esptool-js/>.
3. Baudrate 921600, click "Connect" and pick the serial port.
4. Flash Address `0x0`, choose the file, click "Program".

## C. Espressif Flash Download Tool (Windows)

1. Download <https://dl.espressif.com/public/flash_download_tool.zip>, unpack it
   and start `flash_download_tool_*.exe`.
2. ChipType `ESP32` (or `ESP32-S2` / `ESP32-S3`), WorkMode `Develop`, LoadMode
   `UART`, OK.
3. In the first row: tick the box, choose the `.bin`, address `0x0`. SPI SPEED
   `40MHz`, SPI MODE `DIO`, tick `DoNotChgBin`.
4. Pick the COM port of the drone, BAUD `921600`, click START and wait for
   "FINISH".

## D. esptool on the command line (Linux, macOS, Windows)

```bash
pip install esptool
esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 921600 write_flash 0x0 espdrone-e1-mini-esp32.bin
```

On Windows the port is `COM3` or similar, on macOS `/dev/cu.usbserial-…`.
esptool 5 also accepts `esptool --chip esp32 -p /dev/ttyUSB0 -b 921600 write-flash 0x0 …`.

## E. Build it yourself

```bash
. ~/esp/esp-idf-v5.5.6/export.sh
idf.py set-target esp32        # E1 Mini; esp32s2 / esp32s3 for the ESP32-S2-Drone
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Details and settings: [e1-mini.md](e1-mini.md#build-and-flash).

## If the port does not show up or the connection fails

- Use a USB **data** cable; some cables only charge.
- Hold the BOOT button on the ESP32 while plugging it in (or press RST while
  holding BOOT), then try again.
- Lower the baud rate to 115200.

## How the release is made

A tag `v*` starts [.github/workflows/release.yml](../.github/workflows/release.yml):
GitHub builds the three images with ESP-IDF 5.5, merges each into one file
(`esptool merge_bin`), attaches them to the release and deploys the web flasher
(`tools/web_flasher/`) to GitHub Pages.
