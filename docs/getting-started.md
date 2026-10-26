# Getting started with the E1 Mini

From the box to the first hover in seven steps. Details for every step are in
[e1-mini.md](e1-mini.md) (hardware, bench checklist) and
[webcontrol.md](webcontrol.md) (the control page).

> **Nothing has been tested on real hardware yet.** Keep the propellers off
> until step 6, and stop at the first step that does not behave as described.

## 1. What you need

- The E1 Mini kit ("ESP32 Drone", often with "E1" or "E1 Mini" in the name).
- A 1S LiPo flight battery with a PH2.0 plug, at most 12 mm thick, see
  [Battery](e1-mini.md#battery). The kit usually ships without one.
- A computer with Chrome or Edge and a USB **data** cable, and a phone.

## 2. Back up the factory firmware (optional)

To be able to go back, read the factory firmware first:
`esptool.py --port /dev/ttyUSB0 read_flash 0 ALL e1mini-factory.bin`
(details: [e1-mini.md](e1-mini.md#back-up-the-factory-firmware-before-the-first-flash)).
You can also [try the factory firmware](e1-mini.md#try-the-factory-firmware-first)
first; if the drone flies with it, the hardware is fine.

## 3. Install the firmware

Open the [web flasher](https://technikweber.github.io/esp-drone/) in Chrome or
Edge, connect the ESP32 by USB, click "Install on the E1 Mini", pick the serial
port, tick "Erase device" and wait about a minute. Other ways:
[flashing.md](flashing.md). On Linux your user needs the group `dialout`.

## 4. First start (propellers off)

1. Insert the battery. **Check the polarity first:** pin 1 of the socket is +,
   and there is no protection against a reversed plug.
2. Put the drone on a flat surface and do not touch it.
3. On the phone, join the Wi-Fi `ESP-DRONE_…` (password `12345678`). The
   control page opens by itself; if not, open `http://192.168.43.42`.
4. Wait until "Calibrating – keep the drone still" disappears and "Arm" can be
   pressed. Turn the phone sideways.

## 5. Level the drone

Most IMU modules sit a few degrees off. On a level surface, tap **Trim** in the
top bar, then **Level on the ground**, and keep the drone still for two seconds.
The page shows "Levelled: trim R … · P …"; the drone stores the values. Tap
Trim again to get the horizon back.

## 6. Bench test (propellers still off)

Work through the [bench checklist](e1-mini.md#bench-checklist--without-propellers).
The most important points:

- **Motor order and direction:** front left and rear right spin clockwise, the
  other two counter-clockwise (seen from above).
- **Counter-steering:** hold the drone, throttle down, hold "Arm" for one
  second, throttle to about 20 %. Tilt it right side down: the right motors
  get faster. Nose down: the front motors get faster. If the wrong motors
  react, do not fly.
- **STOP** turns the motors off at once.

## 7. First flight

1. Fit the propellers as from the factory (mind the direction) and, ideally,
   the prop guards. Use a small, open room with nobody nearby.
2. Throttle down, hold "Arm" for one second, then raise the throttle slowly
   until the drone lifts off. Stay low for the first flights.
3. If it drifts steadily to one side, correct it with Trim in 0.5° steps: if it
   drifts to the right, tap ◀.
4. Land by lowering the throttle, then tap "Disarm". If anything goes wrong,
   tap **STOP**.

If the link drops, the drone levels out, descends and turns the motors off
after 4 s. With an empty battery it lands by itself.

## If something is wrong

| What you see | What to do |
|---|---|
| No Wi-Fi `ESP-DRONE_…` | Check the battery and the polarity; flash again. |
| The page does not open | Open `http://192.168.43.42` in the browser; switch off mobile data. |
| "Arm" stays grey | The message line says why: still calibrating (keep it still), battery empty, or another controller is flying. |
| The drone drifts | Level on the ground (step 5), then trim in flight. |
| It oscillates or turns sluggishly | Tune the controller gains, see [First flight](e1-mini.md#first-flight). |
| The motors stop right after take-off | The battery cannot deliver the current; use a 1S pack for RC models. |
| The wrong motors react on the bench | Do not fly; check the motor pins and `IMU_ROTATION` in `menuconfig`. |
