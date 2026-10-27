# Browser control (captive portal + WebSocket)

A phone joins the drone's Wi-Fi, the captive portal window opens by itself and
shows the control page. The page sends commands over a WebSocket (TCP) and gets
telemetry back. No app is needed.

![Control page while flying](images/webcontrol-flying.png)

Code: `components/webctrl/` · page: `components/webctrl/web/index.html` ·
settings: `idf.py menuconfig` → *ESPDrone web control*. Available on all boards
with their own Wi-Fi (transport "native Wi-Fi"), so not on the ESP32-P4.

The page is in English, or in German on German phones; the button at the top
right switches the language. It translates itself because the captive portal
window has no translate function.

## Structure

```
Phone (browser)                         Drone (ESP32)
───────────────                         ─────────────────────────────────────────
DNS: every query   ───── UDP 53 ─────▶  captive DNS: everything → 192.168.43.42
portal check       ───── HTTP :80 ───▶  portal server: unknown URLs → 302 to /
control page (12 KB) ◀── HTTP :80 ────  index.html (gzip, in the firmware)
commands 50 Hz     ───── WS :81 ─────▶  control server ─▶ webctrl_core (failsafe)
telemetry 20 Hz    ◀──── WS :81 ──────                    └▶ commander (priority WEB)
```

- **Separate servers:** port 80 answers captive portal checks and serves the
  page, port 81 carries only the control channel. Each server has its own task,
  so a slow page download on a second phone cannot hold up the pilot's
  commands.
- **CPU cores:** Wi-Fi, lwIP and both servers run on core 0, the sensor and
  stabilizer tasks on core 1.
- **Captive portal:** DNS answers every A query with the drone's address (AAAA
  empty), HTTP redirects all unknown addresses and foreign host names to
  `http://192.168.43.42/`. This triggers the portal detection of Android, iOS
  and Windows. The DHCP server also sets option 114 (RFC 8910) with the API
  address `/captive-portal/api` (RFC 8908).
- The page also works in a normal browser at `http://192.168.43.42`.
- The **desktop ground station** ([groundstation.md](groundstation.md)) uses
  the same port 81: gamepad control, 100 Hz samples of the flight controller,
  controller gains and a motor test. Phone and ground station can be connected
  at the same time; one of them flies.

## Packet loss and delay

Over TCP no command gets lost, but it may arrive too late. After a
retransmission a burst of old commands arrives at once. What helps:

1. **Every command carries the complete control state**, no deltas. A missing
   or dropped command does no harm.
2. **Only the newest counts.** The page sends only when its send buffer is empty
   (`bufferedAmount == 0`) instead of queueing commands. The drone accepts
   commands in sequence order and drops older ones.
3. **Late commands are dropped.** Without a shared clock, the drone compares
   `arrival time − send time` with the smallest value of the last seconds. A
   command more than 250 ms above it is dropped.
4. **Failsafe in stages**, based on the age of the last fresh command:

   | Age | Stage | Behaviour |
   |---|---|---|
   | < 100 ms | OK | the command is executed |
   | 100–300 ms | hold | the last command is held |
   | 300 ms–1 s | level out | roll/pitch 0 (or the trim), no turning, thrust at most the hover thrust |
   | 1–4 s | descend | thrust 85 % of the hover thrust |
   | ≥ 4 s | off | motors off, disarmed |

   The drone learns the **hover thrust** in flight: a low pass over the thrust
   while commands are fresh, valid after 2 s of flight. Without an altitude
   sensor the descent is open loop, like the "Land" failsafe of Betaflight.
5. **React at once:** if the WebSocket closes, the phone leaves the Wi-Fi or the
   page goes to the background (`PAUSE`), the drone jumps straight to "level
   out". The timeline keeps counting from the last fresh command.
6. **Back channel:** the telemetry echoes the sequence number and send time of
   the last accepted command; the page shows the round trip time from it. After
   600 ms without telemetry it warns, after 2.5 s it reconnects.
7. **Resume:** after a dropped connection, also after reloading the page, the
   page comes back with its session token. The drone hands control back without
   arming again. The page restores the throttle only after the drone has
   confirmed the resume.

![Link lost: the drone descends by itself](images/webcontrol-failsafe.png)

All times can be set in `menuconfig`.

## Battery protection

The drone smooths the battery voltage (time constant 0.5 s), because it jumps
with the throttle. Then:

| Situation | Behaviour |
|---|---|
| on the ground below 3.4 V | arming is blocked, the page shows "Battery empty – please charge" |
| in flight below 3.0 V for 1 s | **forced landing:** you keep steering, but the thrust is limited. The limit starts at the hover thrust and goes down to zero within 10 s, then the drone disarms. Throttle all the way down ends the landing at once. |
| armed on the ground below 3.0 V for 1 s | disarmed at once |

ESP-Drone also reports "battery low" when the voltage stays below 3.2 V for
5 s. With the app or cfclient that is the only battery protection; forced
landing and the arming block exist only with web control. Below 0.5 V (no
battery, USB only) the checks are off. Settings in `menuconfig` → *ESPDrone web
control* → *Battery protection*. The values are for a 1S LiPo; compare the
displayed voltage with a multimeter first.

As with a lost link, the forced landing is open loop without an altitude sensor.
From 1–2 m the drone touches down after about 2 s at 1.5–2.5 m/s.

![Forced landing with an empty battery](images/webcontrol-battery.png)

## Trim

If the drone drifts sideways by itself while hovering, the trim corrects it: tap
"Trim" in the top bar, then counter-steer with the arrows in 0.5° steps, up to
±5°. If it drifts to the right, tap ◀. A long press on the middle resets to 0.

- The trim is added to roll and pitch; the sum stays within the maximum angle.
  In the failsafe the drone levels out to the trim instead of 0°.
- The **drone** stores the values in NVS, not the phone: they belong to the
  drone, and the portal window on iPhones forgets its storage. They are written
  only while nobody is armed, because writing the flash stalls the flight
  controller briefly.
- **Level on the ground** (button in the trim panel): with the drone standing
  still on a level surface, the page averages the attitude the drone reports
  for 2 s and sets the trim from it, so the drone hovers level from the first
  flight. It refuses if the drone moves or is tilted more than 5°, and works
  only while disarmed.
- On the ground every phone may trim, in flight only the pilot.
- The trim applies to web control only, not to the app or cfclient.

![Trim panel](images/webcontrol-trim.png)

## Arming and pilots

- The drone starts **disarmed**. Only one source arms at a time: the web pilot,
  a CRTP pilot (app, cfclient, ESP-NOW joystick) or `system.forceArm`.
- Web: throttle all the way down, hold "Arm" for 1 s. The self test must have
  passed and the sensors must be calibrated, the battery must not be empty and
  the drone must not be upside down.
- After STOP or a failsafe, the page has to release ARM once before it can arm
  again. The same applies to every new connection.
- **STOP** can be triggered from every connected phone. It disarms every pilot,
  including the app or cfclient.
- Further phones see the telemetry but do not control.
- When the drone stops, the page shows why: STOP, link lost, upside down or
  battery empty.

## Flips (experimental)

The pilot taps a flip button, the drone flies the flip by itself, so a stalled
link cannot cut it in half. The buttons are hidden: *Info* → *Experimental* →
"Show flip buttons" on the phone, *Controller* → *Experimental* in the
[ground station](groundstation.md#flips-experimental).

1. **Climb:** 250 ms at 170 % of the hover thrust.
2. **Turn** about the roll or pitch axis at 800 °/s with 50 % of the hover
   thrust (upside down, thrust pushes the drone down). Both tilt axes are rate
   controlled, because the Euler angles of the attitude controller jump by
   180° when a pitch flip passes the vertical; yaw holds where it is.
3. Once the drone has been upside down and is back within 70° of upright, the
   attitude controller **levels it out** (trim) with 140 % of the hover thrust
   for 350 ms to catch the fall. If it has not turned over after 900 ms, it
   levels out without finishing.
4. Then the pilot's sticks count again.

The drone only flips for the web pilot, in flight after it has learned the
hover thrust (about 2 s of flying), with the throttle near hover, a fresh link,
no forced landing and no motor test. Meanwhile the telemetry shows state 2
bit 3. The STOP button works as always.

**Without an altitude sensor this is open loop:** how much height the drone
loses depends on the battery, the weight and the values above. Start at 2 m or
higher, with plenty of room. Not tested on real hardware yet. The values are
the defaults in `menuconfig` → *ESPDrone web control* → *Flips (experimental)*,
which can also switch flips off completely; the ground station can change and
store them while the drone is on the ground (parameter group `flip`).

## Protocol

Binary WebSocket frames, little endian. Defined in
`components/webctrl/include/webctrl_core.h`.

- Version 2: TRIM message, telemetry with 40 instead of 36 bytes (battery state,
  stop reason, trim).
- Version 3: STREAM, PARAM and MOTOR_TEST for the ground station, FAST samples,
  motor test bit in the telemetry. All of it is optional: a page that only knows
  version 2 works unchanged, and the drone ignores unknown message types.
- Version 4: FLIP (experimental), flip parameters writable with PARAM, `flip`
  in the info frame, flip bit in the telemetry.

### CONTROL (page → drone, 16 bytes, 50 Hz)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x01` |
| 1 | u8 | flags: bit0 ARM, bit1 KILL, bit2 PAUSE |
| 2 | u16 | sequence number (wraps) |
| 4 | u32 | send time of the page in ms (`performance.now()`) |
| 8 | i16 | roll −1000…1000 (+ = right) |
| 10 | i16 | pitch −1000…1000 (+ = forward) |
| 12 | i16 | yaw −1000…1000 (+ = turn right) |
| 14 | u16 | throttle 0…1000 |

Full stick maps to ±15° and ±120 °/s (configurable). Invalid frames (length,
type, value range, unknown flags) are ignored.

### HELLO (page → drone, 8 bytes, right after connecting)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x02` |
| 1 | u8 | protocol version (4) |
| 4 | u32 | session token, 0 = new session |

The drone answers with a text frame `{"t":"info", …}` (firmware, device,
`proto`, limits, failsafe times, battery limits `battery`, `trimMax`,
`resumed`; since version 3 also `fastHz` and `motorTest` with `maxPercent` and
`maxMs`, `maxPercent` 0 = no motor test; since version 4 `flip`, true if flips
are compiled in). Telemetry starts only after the HELLO.

### TRIM (page → drone, 4 bytes, on every change)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x03` |
| 1 | i8 | roll trim in 1/10° (+ = right), −50…50 |
| 2 | i8 | pitch trim in 1/10° (+ = forward), −50…50 |
| 3 | u8 | reserved (0) |

Absolute values. On the ground the drone takes them from every phone, in flight
only from the pilot, and reports the valid values back in the telemetry.

### TELEMETRY (drone → page, 40 bytes, 20 Hz)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x81` |
| 1 | u8 | state: bit0 armed, bit1 you are the pilot, bit2 other controller, bit3 failsafe, bit4 battery low, bit5 release ARM first, bit6 ready, bit7 upside down |
| 2 | u8 | failsafe stage 0…4 |
| 3 | u8 | result of the last arming attempt: 0 OK, 1 not ready, 2 throttle not down, 3 battery, 4 other controller, 5 upside down, 6 release ARM first, 7 motor test running |
| 4 | u16 | sequence number of the last accepted command |
| 6 | u16 | accepted commands per second |
| 8 | u32 | send time of that command (echo → round trip time) |
| 12 | u16 | battery voltage in mV |
| 14, 16, 18 | i16 | roll, pitch, yaw in 1/100° (pitch + = nose up) |
| 20 | u16 | thrust sent to the controller 0…1000 |
| 22 | u16 | dropped late commands |
| 24 | u16 | extra delay of the last command in ms |
| 26 | u8 | state 2: bit0 forced landing (battery empty), bit1 battery too low to arm (only on the ground), bit2 motor test running, bit3 flip running |
| 27 | u8 | reason of the last stop: 0 none, 1 disarmed, 2 STOP, 3 link lost, 4 upside down, 5 battery empty, 6 disarmed elsewhere |
| 28 | u32 | uptime of the drone in ms |
| 32 | u32 | session token (to the pilot only) |
| 36, 37 | i8 | roll and pitch trim in 1/10° |
| 38 | u16 | reserved |

### STREAM (client → drone, 4 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x05` |
| 1 | u8 | flags: bit0 FAST samples on |
| 2 | u16 | reserved (0) |

Off for every new connection; the phone page never asks for FAST samples.

### FAST (drone → client, 12 + 32 × n bytes, 20 frames per second)

Samples of the flight controller every 10 ms, five per frame, only to clients
that switched them on. Lost frames show as a gap in the sample numbers.

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x82` |
| 1 | u8 | number of samples n (1…5) |
| 2 | u16 | number of the first sample (wraps) |
| 4 | u32 | drone time of the first sample in ms |
| 8 | u8 | time between samples in ms (10) |
| 9 | 3 bytes | reserved |
| 12 + 32·i | | sample i: |
| +0, +2, +4 | i16 | roll, pitch, yaw in 1/100° (as in the telemetry) |
| +6, +8 | i16 | roll and pitch setpoint of the attitude controller in 1/100° |
| +10, +12, +14 | i16 | gyro roll, pitch, yaw in 1/10 °/s |
| +16, +18, +20 | i16 | rate setpoints roll, pitch, yaw in 1/10 °/s |
| +22 … +28 | u16 | motor outputs M1…M4, 0…65535 |
| +30 | u16 | thrust to the mixer, 0…65535 (web control: at most 60000) |

Rates are in the convention of the rate controller: roll + = to the right,
pitch + = nose up, yaw + = counter-clockwise seen from above.

### PARAM (client → drone, 8 + name bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x04` |
| 1 | u8 | operation: 0 read, 1 write, 2 save, 3 reset |
| 2 | u8 | request id (echoed) |
| 3 | u8 | reserved (0) |
| 4 | f32 | value (write) |
| 8 | ASCII | parameter `group.name`, up to 31 characters, no terminator (read and write only) |

Every firmware parameter can be read. Only these can be written; they take
effect at once:

- the 18 controller gains `pid_rate.*` and `pid_attitude.*` (`roll_kp` …
  `yaw_kd`), 0 to 5000;
- the flip parameters (version 4): `flip.rate` 360–2000 °/s, `flip.boostMs`
  0–600, `flip.boost` 100–250 % of hover, `flip.spin` 10–150 %,
  `flip.exitDeg` 10–120°, `flip.recoverMs` 0–1000, `flip.recover` 50–250 %,
  `flip.timeoutMs` 300–1500 (not while a flip runs).

In flight only the pilot may write. *Save* stores all of them in the drone's
flash (NVS), where they replace the firmware defaults at the next start;
*reset* sets the defaults again and erases the stored values. Both only while
nobody flies, because writing the flash pauses the flight controller for a few
milliseconds.

### PARAM_REPLY (drone → client, 14 + name bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x84` |
| 1 | u8 | operation |
| 2 | u8 | request id |
| 3 | u8 | status: 0 OK, 1 unknown parameter, 2 not writable, 3 busy (someone flies, saving in flight, starting up), 4 value out of range, 5 flash error |
| 4 | f32 | current value |
| 8 | f32 | firmware default of a gain, NaN otherwise |
| 12 | u8 | flags: bit0 writable (a gain), bit1 stored gains are in use |
| 13 | u8 | reserved |
| 14 | ASCII | parameter name |

### MOTOR_TEST (client → drone, 8 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x06` |
| 1 | u8 | motors: bit0 M1 … bit3 M4; 0 = stop |
| 2 | u8 | power in % (1…100) |
| 3 | u8 | reserved (0) |
| 4 | u16 | duration in ms |
| 6 | u16 | reserved (0) |

For the bench with the propellers off. The drone only starts it while nobody
flies and the sensors are calibrated, and caps power
(`WEBCTRL_MOTOR_TEST_MAX_PERCENT`, default 30 %, 0 = off) and duration (3 s).
It stops on time, on STOP from any client, and when the starting client
disconnects. Meanwhile nobody can arm (arm result 7). The motors get their
ratio through the parameter group `motorPowerSet` while the drone is armed for
the test.

### MOTOR_REPLY (drone → client, 8 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x86` |
| 1 | u8 | motors as applied |
| 2 | u8 | power in % as applied |
| 3 | u8 | status: 0 OK, 1 not ready, 2 busy (armed or someone flies), 3 switched off |
| 4 | u16 | duration in ms as applied |
| 6 | u16 | reserved |

### FLIP (client → drone, 4 bytes, experimental)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x07` |
| 1 | u8 | direction: 0 forward (nose down first), 1 backward, 2 left, 3 right |
| 2 | u16 | reserved (0) |

### FLIP_REPLY (drone → client, 4 bytes)

| Offset | Type | Field |
|---|---|---|
| 0 | u8 | `0x87` |
| 1 | u8 | direction |
| 2 | u8 | status: 0 flip started, 1 switched off, 2 not the pilot, 3 not flying (hover thrust not learned yet, or throttle too low), 4 busy (flip running, failsafe, forced landing, motor test) |
| 3 | u8 | reserved |

## Testing without a drone

- **Unit tests** for the protocol (including the ground station messages),
  failsafe, delay estimator, resume, trim, battery protection, gains, motor
  test checks, the flip sequence and DNS: `make -C components/webctrl/host_test`
- **Simulated drone** for the page:
  `python3 tools/mock_drone.py --stall-every 6 --stall-ms 450`, then open
  `http://127.0.0.1:8080/?host=127.0.0.1:8081`. The option creates a TCP-like
  stall of 450 ms every 6 s. With `--vbat 3.45 --drain 0.02` the simulated
  battery starts nearly empty: after about 10 s of hovering the forced landing
  begins. `&lang=de` or `&lang=en` in the address picks the language. The
  mock simulates the PID controller and the airframe (`--turbulence` sets the
  gusts) and answers PARAM, STREAM, MOTOR_TEST and FLIP; `--legacy` makes it
  behave like firmware v0.2.
- **Screenshots** for the README: `python3 tools/ui_screenshots.py` drives the
  page in headless Google Chrome against the simulated drone and rewrites
  `docs/images/webcontrol-*.png` (ImageMagick makes them smaller, if
  installed). `python3 tools/groundstation/screenshots.py` does the same for
  the ground station, with a simulated gamepad.

## Limits

- **HTTP only**; a valid HTTPS certificate is not possible in the drone's
  Wi-Fi. Steering by tilting the phone, the Gamepad API and the Wake Lock API
  need HTTPS and are therefore missing on the phone page. Gamepads work in the
  desktop ground station, which runs on `localhost` ([groundstation.md](groundstation.md)).
- **Portal window:** iOS and Android open the page in a mini browser. On iOS
  "Cancel" may disconnect the Wi-Fi; Android may route normal apps over mobile
  data when there is "no internet". If in doubt, open the page in the normal
  browser at `http://192.168.43.42`. Not yet tested on real devices.
- **Descending without an altitude sensor** is open loop (estimated hover
  thrust), both after a lost link and in a forced landing.
- The Wi-Fi password is `12345678` by default. Anyone who joins can watch and
  trigger STOP. Before flying among strangers, change `WIFI_PASSWORD` in
  `menuconfig`.
