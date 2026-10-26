/*
 * ESP-Drone Firmware
 *
 * Copyright 2026 Philipp Weber
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * webctrl.h - control the drone from a browser: captive portal on port 80,
 * WebSocket control channel on port 81, link supervision with failsafe.
 */

#ifndef WEBCTRL_H_
#define WEBCTRL_H_

/*
 * Start the web control task. It waits until the Wi-Fi access point and the
 * commander are up, then starts the captive portal and the control server.
 * Call once from app_main after systemLaunch().
 */
void webctrlLaunch(void);

#endif /* WEBCTRL_H_ */
