# ESP-Drone auf dem CHERGLOW E1 Mini

Diese Firmware-Variante läuft auf dem Bausatz **CHERGLOW (琛光) E1 Mini**
(Open-Hardware-Projekt: <https://oshwhub.com/songge8/project_qqqyfdkm>), der unter
anderem als „ESP32 Drone Kit“ verkauft wird. Ab Werk läuft darauf die
Arduino-Firmware *CF-Drone* ([songge8/CF-Drone](https://github.com/songge8/CF-Drone)),
bei manchen Händlern umbenannt in „XW-Drone“. Diese Anleitung beschreibt, wie
ESP-Drone an die Hardware angepasst wurde, wie man flasht und wie man vor dem
ersten Flug am Prüfstand testet.

> **Stand:** portiert anhand der Herstellerunterlagen, noch **nicht auf echter
> Hardware getestet**. Die Prüfstand-Checkliste unten ist deshalb Pflicht.

## Hardware

| Funktion | Anschluss | Quelle |
|---|---|---|
| Controller | ESP32-DevKit V1, 30 Pins, klassischer ESP32 (kein S3) | Projektseite, Fotos |
| Lagesensor | MPU6500 per SPI: SCK GPIO18, MISO GPIO19 (Modul-Pin AD0), MOSI GPIO23 (SDA), CS GPIO5 (NCS), **kein Interrupt** | CF-Drone `board_config.h`, `imu.ino` |
| Motor M1 vorne rechts | GPIO15 | CF-Drone `BOARD_MOTOR_PINS` |
| Motor M2 hinten rechts | GPIO13 | |
| Motor M3 hinten links | GPIO12 | |
| Motor M4 vorne links | GPIO14 | |
| Motortreiber | AO3416-MOSFETs, 25 kHz PWM | Stückliste, CF-Drone `motors.ino` |
| Akkuspannung | GPIO36, Teiler 10 kΩ / 33 kΩ → Vbat = Vadc × 1,303 | CF-Drone `battery.ino` |
| Status-LED | GPIO2 (blaue LED des DevKits) | CF-Drone |
| Erweiterung | I2C SDA 21 / SCL 22, PMW3901-CS GPIO33, VL53L1X-XSHUT GPIO32 | CF-Drone |
| Buzzer | keiner | |

Die Pin-Nummern sind Fakten aus den Herstellerdateien; aus dem CF-Drone-Code
wurde nichts übernommen.

### Einbaulage des Lagesensors

ESP-Drone rechnet im Körpersystem der Crazyflie: X nach vorn, Y nach links,
Z nach oben. Die Werks-Firmware stellt `imuRotation = (0, 0, −90°)` ein und
beschreibt die Einbaulage im Kommentar dazu (`imu.ino`) ausdrücklich: Chip oben,
Sensor-X zeigt nach rechts, Sensor-Y nach vorn, umgerechnet `(y, −x, z)`. Also:

- Sensor-Y zeigt nach vorn → Körper-X = Sensor-Y
- Sensor-X zeigt nach rechts → Körper-Y = −Sensor-X
- Sensor-Z zeigt nach oben

In `menuconfig` ist das **„270 deg: body X = sensor Y, body Y = −sensor X“**
(`CONFIG_IMU_ROTATION_270`, Standard für den E1 Mini). Am Prüfstand prüfen!

### Drehrichtung der Motoren

Aus dem Mischer der Werks-Firmware: **vorne links und hinten rechts drehen im
Uhrzeigersinn**, vorne rechts und hinten links gegen den Uhrzeigersinn (von oben
gesehen). Der Crazyflie-Mischer von ESP-Drone erwartet dasselbe, deshalb ist
`MOTOR_YAW_INVERT` aus.

## Was gegenüber ESP-Drone geändert wurde

- **Neues Board** `TARGET_E1_MINI` mit allen Pins (`menuconfig` → ESPDrone Config).
- **MPU6500-Treiber per SPI** (`components/drivers/spi_devices/mpu6500`) und
  Sensor-Schicht `sensors_mpu6500_spi.c`. Ohne Interrupt-Leitung gibt ein
  Hardware-Timer den 1-kHz-Takt vor. Sensor- und Regel-Task laufen auf
  CPU-Kern 1, WLAN und Webserver auf Kern 0.
- **Gemeinsamer SPI-Bus** für Lagesensor und optionalen Flow-Sensor (`spi_bus`).
- **Motor-PWM** 25 kHz / 10 Bit, Motor-Pins werden beim Start sofort auf LOW
  gezogen. Den kurzen Motortest beim Start (`MOTORS_STARTUP_TEST`) hat Espressif
  standardmäßig abgeschaltet; für den Prüfstand kann man ihn einschalten.
- **Start gesperrt (disarmed).** Freischalten nur durch genau eine Quelle:
  Web-Steuerung (Taste), App/cfclient/ESP-NOW (Gas einmal auf null) oder den
  Parameter `system.forceArm`.
- **Brownout-Reset aus**, wie in der Werks-Firmware: Beim Anlaufen der Motoren
  bricht die 3,3-V-Versorgung kurz ein, ein Reset in der Luft wäre schlimmer.
- **Partitionstabelle** für 4 MB mit zwei App-Bereichen (späteres Update über
  WLAN) und Core-Dump-Bereich.
- **Browser-Steuerung mit Captive Portal**, siehe [webcontrol.md](webcontrol.md).

## Werkzeuge einrichten

ESP-IDF **v5.5** (hier v5.5.6 mit Python 3.14 unter Fedora 44). v6.1 geht
ebenfalls, GitHub Actions baut jede Änderung mit beiden Versionen:

```bash
git clone -b v5.5.6 --depth 1 --recursive --shallow-submodules \
    https://github.com/espressif/esp-idf.git ~/esp/esp-idf-v5.5.6
~/esp/esp-idf-v5.5.6/install.sh esp32
. ~/esp/esp-idf-v5.5.6/export.sh
```

Zugriff auf den USB-Seriell-Wandler des DevKits (`/dev/ttyUSB0`) braucht unter
Fedora die Gruppe `dialout`:

```bash
sudo usermod -aG dialout $USER     # danach ab- und wieder anmelden
```

## Werks-Firmware zuerst ausprobieren

Bevor du etwas änderst: Fliegt die Drohne mit der Werks-Firmware, ist die
Hardware in Ordnung, und du hast einen Vergleich für später.

- **Handy-Steuerung ab Werk:** WLAN `Drone_WiFi`, Passwort `12345678`, dann im
  Browser `http://192.168.4.1` öffnen (die Seite öffnet sich nicht von selbst).
- **Serielle Konsole**, Propeller ab:

  ```bash
  . ~/esp/esp-idf-v5.5.6/export.sh
  python -m serial.tools.miniterm --eol LF /dev/ttyUSB0 115200   # beenden: Strg+]
  ```

  - `imu` zeigt den Lagesensor, erwartet: `model: MPU-6500` und `who am I: 0x70`
  - `mfl`, `mfr`, `mrl`, `mrr` lassen je einen Motor 3 s mit 30 % laufen (vorne
    links, vorne rechts, hinten links, hinten rechts). Prüfen: richtige Position,
    **vorne links und hinten rechts drehen im Uhrzeigersinn**, die anderen
    gegen den Uhrzeigersinn (von oben gesehen).

Die fertige Werks-Firmware (`XW-Drone.ino.merged.bin`) bietet der Händler nur
über Baidu an (Konto nötig). Am einfachsten geht es mit der eigenen Sicherung
aus dem nächsten Schritt zurück.

## Werks-Firmware sichern (vor dem ersten Flashen!)

Die Werks-Firmware ist auf die Drohne abgestimmt. Sichere den ganzen Flash,
dann kannst du jederzeit zurück. **Propeller vorher abnehmen.**

```bash
. ~/esp/esp-idf-v5.5.6/export.sh
esptool.py --port /dev/ttyUSB0 chip_id          # muss "ESP32" melden
esptool.py --port /dev/ttyUSB0 flash_id         # zeigt die Flash-Größe (4 MB erwartet)
esptool.py --port /dev/ttyUSB0 -b 921600 read_flash 0 ALL e1mini-werksfirmware.bin
```

Die Datei muss so groß sein wie der Flash (4 MB = 4194304 Bytes). Zurückspielen:

```bash
esptool.py --port /dev/ttyUSB0 -b 921600 write_flash 0 e1mini-werksfirmware.bin
```

## Bauen und flashen

```bash
. ~/esp/esp-idf-v5.5.6/export.sh
idf.py set-target esp32        # wählt automatisch das Board E1 Mini
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Einstellungen (Pins, Failsafe-Zeiten, WLAN-Name/Passwort) über `idf.py menuconfig`
→ *ESPDrone Config* und *ESPDrone web control*.

## Prüfstand-Checkliste — OHNE Propeller

Jeder Punkt muss stimmen, bevor Propeller montiert werden.

1. **Startprotokoll** (`idf.py monitor`):
   - `MPU6500 SPI connection [OK], WHO_AM_I = 0x70` (0x71/0x73 bei MPU9250-Varianten)
   - `IMU sample timer running at 1000 Hz on core 1`
   - `web control ready: http://192.168.43.42/`
   - kein Reset, kein `assert`; die LED blinkt im „alive“-Muster
2. **Motor-Reihenfolge:** In `menuconfig` → *motors config* die Option
   „Spin motors during startup self-test“ einschalten (standardmäßig aus) und
   neu flashen. Beim Start laufen die Motoren dann nacheinander kurz an:
   M1 vorne rechts → M2 hinten rechts → M3 hinten links → M4 vorne links.
   Stimmt die Position nicht: Pins in `menuconfig` → *motors config* tauschen.
   Danach die Option wieder ausschalten.
3. **Lage:** Handy mit dem WLAN `ESP-DRONE_…` verbinden (Passwort `12345678`),
   die Steuerseite öffnet sich. Drohne flach hinlegen und still liegen lassen,
   bis „Bereit“ erscheint. Dann:
   - Nase nach unten kippen → **N wird negativ**
   - rechte Seite nach unten kippen → **R wird positiv**
   - Stimmen Vorzeichen oder Achsen nicht: `IMU_ROTATION` in `menuconfig` ändern.
4. **Gegensteuern (wichtigster Test):** Drohne in der Hand halten, Gas runter,
   scharfschalten, Gas auf ca. 20 %:
   - rechts nach unten kippen → die **rechten** Motoren drehen hörbar schneller
   - Nase nach unten kippen → die **vorderen** Motoren drehen schneller
   - im Uhrzeigersinn drehen → **vorne links und hinten rechts** drehen schneller
   - Reagieren die falschen Motoren: **nicht fliegen**, Einbaulage bzw.
     `MOTOR_YAW_INVERT` prüfen.
5. **Failsafe:** scharf, Gas 30 %, dann am Handy das WLAN ausschalten. Nach
   0,3 s gleichen sich die Motoren an (gerade ausrichten), nach 1 s wird es
   leiser (sinken), nach 4 s stehen die Motoren. Im Protokoll erscheinen
   `link failsafe: stage …` und `disarmed: link lost …`.
6. **NOT-AUS:** scharf, Gas 30 %, NOT-AUS tippen → Motoren stehen sofort.
7. **Akku:** Spannung auf der Seite mit einem Multimeter am Akku vergleichen.
   Bei Abweichung `BATTERY_DIVIDER_X1000` anpassen. Hinweis: Über ca. 4,05 V
   kann der ESP32-ADC wegen des Spannungsteilers nicht mehr genau messen.

## Erster Flug

- Kleiner, freier Raum, keine Personen in der Nähe, Prop-Guards montiert.
- Propeller wie ab Werk (Drehrichtung beachten), Akku voll.
- Drohne eben hinstellen, bis „Bereit“ still liegen lassen (Kalibrierung).
- Scharfschalten, Gas langsam erhöhen. Kippt oder dreht sie beim Abheben weg:
  sofort **NOT-AUS**.
- Die Reglerwerte sind die ESP-Drone-Standardwerte (Drehrate Roll/Nick: P 250,
  I 500, D 2,5; Gier P 120; Lage P 5,9). Die Werks-Firmware nutzt für diesen
  Rahmen, in ESP-Drone-Einheiten umgerechnet, etwa Drehrate P 137, I 229, D 2,3,
  Gier P 343 und Lage P 6. Schwingt der E1 Mini oder dreht er träge um die
  Hochachse, die Parameter `pid_rate.*` in diese Richtung ändern (per cfclient
  über UDP). Umrechnung: Wert × 2 · 65535 / 57,3 für Roll und Nick (der
  ESP-Drone-Mischer halbiert diese Anteile), × 65535 / 57,3 für Gier.
