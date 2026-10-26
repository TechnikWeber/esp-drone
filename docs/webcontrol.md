# Browser-Steuerung (Captive Portal + WebSocket)

Ein Handy verbindet sich mit dem WLAN der Drohne, das Captive-Portal-Fenster
öffnet sich von selbst und zeigt die Steuerseite. Die Seite schickt
Steuerbefehle über einen WebSocket (TCP) und bekommt Telemetrie zurück. Es
braucht keine App.

Code: `components/webctrl/` · Seite: `components/webctrl/web/index.html` ·
Einstellungen: `idf.py menuconfig` → *ESPDrone web control*. Verfügbar auf allen
Boards mit eigenem WLAN (Transport „native Wi-Fi“), also nicht auf dem ESP32-P4.

## Aufbau

```
Handy (Browser)                         Drohne (ESP32)
───────────────                         ─────────────────────────────────────────
DNS: jede Anfrage  ───── UDP 53 ─────▶  Captive-DNS: alles → 192.168.43.42
Portal-Prüfung     ───── HTTP :80 ───▶  Portal-Server: unbekannte URLs → 302 auf /
Steuerseite (7 KB) ◀──── HTTP :80 ────  index.html (gzip, in der Firmware)
Steuerbefehle 50 Hz ──── WS :81 ─────▶  Control-Server ─▶ webctrl_core (Failsafe)
Telemetrie 20 Hz   ◀──── WS :81 ──────                    └▶ Commander (Prio WEB)
```

- **Getrennte Server:** Port 80 bedient Captive-Portal-Prüfungen und lädt die
  Seite aus, Port 81 nur die Steuerung. Jeder Server hat einen eigenen Task, so
  kann ein langsamer Seitenabruf eines zweiten Handys die Befehle des Piloten
  nicht aufhalten.
- **CPU-Kerne:** WLAN, lwIP und beide Server laufen auf Kern 0, Sensor- und
  Regel-Task auf Kern 1.
- **Captive Portal:** DNS antwortet jede A-Anfrage mit der Drohnen-IP (AAAA
  leer), HTTP leitet alle unbekannten Adressen und fremden Host-Namen auf
  `http://192.168.43.42/` um. Das löst die Portal-Erkennung von Android, iOS
  und Windows aus. Zusätzlich setzt der DHCP-Server Option 114 (RFC 8910) mit
  der API-Adresse `/captive-portal/api` (RFC 8908).
- Die Seite läuft auch im normalen Browser unter `http://192.168.43.42`.

## Umgang mit Paketverlust und Verzögerung

Über TCP geht kein Befehl verloren, er kommt aber womöglich zu spät. Nach einer
Neuübertragung trifft ein Schwall alter Befehle auf einmal ein. Dagegen:

1. **Jeder Befehl enthält den kompletten Steuerzustand**, keine Änderungen.
   Ein fehlender oder verworfener Befehl schadet nicht.
2. **Nur der neueste zählt.** Die Seite sendet nur, wenn ihr Sendepuffer leer
   ist (`bufferedAmount == 0`), statt Befehle anzustauen. Die Drohne übernimmt
   Befehle in Sequenz-Reihenfolge und verwirft ältere.
3. **Verspätete Befehle werden verworfen.** Ohne gemeinsame Uhr vergleicht die
   Drohne `Ankunftszeit − Sendezeit` mit dem kleinsten Wert der letzten
   Sekunden. Liegt ein Befehl mehr als 250 ms darüber, wird er verworfen.
4. **Failsafe in Stufen**, gemessen am Alter des letzten frischen Befehls:

   | Alter | Stufe | Verhalten |
   |---|---|---|
   | < 100 ms | OK | Befehl wird ausgeführt |
   | 100–300 ms | Halten | letzter Befehl wird gehalten |
   | 300 ms–1 s | Ausrichten | Rollen/Nicken 0, keine Drehung, Schub höchstens Schwebeschub |
   | 1–4 s | Sinken | Schub 85 % des Schwebeschubs |
   | ≥ 4 s | Aus | Motoren aus, entschärft |

   Den **Schwebeschub** lernt die Drohne im Flug: Tiefpass über den Schub, solange
   Befehle frisch sind; gültig nach 2 s Flugzeit. Ohne Höhensensor ist das
   Sinken ungeregelt, wie der „Land“-Failsafe von Betaflight.
5. **Sofort reagieren:** Wird der WebSocket geschlossen, verlässt das Handy das
   WLAN oder geht die Seite in den Hintergrund (`PAUSE`), springt die Drohne
   direkt auf „Ausrichten“. Die Zeitlinie zählt weiter ab dem letzten frischen
   Befehl.
6. **Rückkanal:** Die Telemetrie spiegelt Sequenznummer und Sendezeit des
   letzten übernommenen Befehls; die Seite zeigt daraus die Latenz. Kommt
   600 ms keine Telemetrie, warnt sie; nach 2,5 s baut sie die Verbindung neu
   auf.
7. **Wiederaufnahme:** Nach einem Verbindungsabbruch, auch nach einem
   Neuladen der Seite, meldet sich die Seite mit ihrem Sitzungs-Token. Die
   Drohne gibt ihr die Steuerung zurück, ohne neu scharfzuschalten. Den
   Gasstand stellt die Seite erst wieder her, wenn die Drohne die Wiederaufnahme
   bestätigt hat.

Alle Zeiten sind in `menuconfig` einstellbar.

## Scharfschalten und Piloten

- Die Drohne startet **gesperrt**. Scharf schaltet immer nur eine Quelle:
  Web-Pilot, CRTP-Pilot (App, cfclient, ESP-NOW-Joystick) oder `system.forceArm`.
- Web: Gas ganz unten, „Scharfschalten“ 1 s halten. Vorher müssen der
  Selbsttest bestanden und die Sensoren kalibriert sein, der Akku darf nicht
  leer sein und die Drohne nicht auf dem Kopf liegen.
- Nach NOT-AUS oder Failsafe muss die Seite ARM erst einmal loslassen, bevor neu
  geschärft werden kann. Das gilt auch für jede neue Verbindung.
- **NOT-AUS** darf jedes verbundene Handy auslösen. Er entschärft jeden
  Piloten, auch App oder cfclient.
- Weitere Handys sehen die Telemetrie, steuern aber nicht.

## Protokoll

Binäre WebSocket-Frames, Little Endian. Definiert in
`components/webctrl/include/webctrl_core.h`.

### CONTROL (Seite → Drohne, 16 Bytes, 50 Hz)

| Offset | Typ | Feld |
|---|---|---|
| 0 | u8 | `0x01` |
| 1 | u8 | Flags: bit0 ARM, bit1 KILL, bit2 PAUSE |
| 2 | u16 | Sequenznummer (läuft über) |
| 4 | u32 | Sendezeit der Seite in ms (`performance.now()`) |
| 8 | i16 | Rollen −1000…1000 (+ = rechts) |
| 10 | i16 | Nicken −1000…1000 (+ = vorwärts) |
| 12 | i16 | Gieren −1000…1000 (+ = rechts drehen) |
| 14 | u16 | Gas 0…1000 |

Ausschläge werden auf ±15° bzw. ±120 °/s abgebildet (einstellbar). Ungültige
Frames (Länge, Typ, Wertebereich, unbekannte Flags) werden ignoriert.

### HELLO (Seite → Drohne, 8 Bytes, direkt nach dem Verbinden)

| Offset | Typ | Feld |
|---|---|---|
| 0 | u8 | `0x02` |
| 1 | u8 | Protokollversion (1) |
| 4 | u32 | Sitzungs-Token, 0 = neue Sitzung |

Die Drohne antwortet mit einem Text-Frame `{"t":"info", …}` (Firmware,
Gerät, Grenzwerte, `resumed`). Telemetrie gibt es erst nach dem HELLO.

### TELEMETRY (Drohne → Seite, 36 Bytes, 20 Hz)

| Offset | Typ | Feld |
|---|---|---|
| 0 | u8 | `0x81` |
| 1 | u8 | Status: bit0 scharf, bit1 du bist Pilot, bit2 andere Steuerung, bit3 Failsafe, bit4 Akku schwach, bit5 ARM erst loslassen, bit6 bereit, bit7 auf dem Kopf |
| 2 | u8 | Failsafe-Stufe 0…4 |
| 3 | u8 | Ergebnis des letzten Scharfschalt-Versuchs |
| 4 | u16 | Sequenznummer des letzten übernommenen Befehls |
| 6 | u16 | übernommene Befehle pro Sekunde |
| 8 | u32 | Sendezeit dieses Befehls (Echo → Latenz) |
| 12 | u16 | Akkuspannung in mV |
| 14, 16, 18 | i16 | Rollen, Nicken, Gieren in 1/100° (Nicken + = Nase hoch) |
| 20 | u16 | Schub an den Regler 0…1000 |
| 22 | u16 | verworfene verspätete Befehle |
| 24 | u16 | Zusatzverzögerung des letzten Befehls in ms |
| 28 | u32 | Laufzeit der Drohne in ms |
| 32 | u32 | Sitzungs-Token (nur an den Piloten) |

## Testen ohne Drohne

- **Unit-Tests** für Protokoll, Failsafe, Verzögerungsschätzer, Wiederaufnahme
  und DNS: `make -C components/webctrl/host_test`
- **Simulierte Drohne** für die Seite:
  `python3 tools/mock_drone.py --stall-every 6 --stall-ms 450`, dann
  `http://127.0.0.1:8080/?host=127.0.0.1:8081` öffnen. Die Option erzeugt alle
  6 s einen TCP-artigen Hänger von 450 ms.

## Grenzen

- **Nur HTTP**, ein gültiges HTTPS-Zertifikat ist im Drohnen-WLAN nicht
  möglich. Steuern durch Kippen des Handys, die Gamepad-API und die Wake-Lock-API
  brauchen HTTPS und fehlen deshalb.
- **Portal-Fenster:** iOS und Android öffnen die Seite in einem Mini-Browser.
  Bei iOS kann „Abbrechen“ das WLAN trennen; Android leitet normale Apps bei
  „kein Internet“ eventuell über mobile Daten. Im Zweifel die Seite im
  normalen Browser unter `http://192.168.43.42` öffnen. Auf echten Geräten
  noch ungetestet.
- **Sinken ohne Höhensensor** ist ungeregelt (geschätzter Schwebeschub).
- Das WLAN-Passwort ist standardmäßig `12345678`. Wer sich verbindet, kann
  zuschauen und NOT-AUS auslösen. Für den Einsatz unter Fremden
  `WIFI_PASSWORD` in `menuconfig` ändern.
