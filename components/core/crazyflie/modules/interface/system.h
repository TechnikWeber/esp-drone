/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * system.h - Top level module header file
 */

#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#include <stdbool.h>
#include <stdint.h>

void systemInit(void);
bool systemTest(void);

void systemLaunch(void);


void systemStart();
void systemWaitStart(void);
void systemSetCanFly(bool val);
bool systemCanFly(void);
void systemSetArmed(bool val);
bool systemIsArmed();

/* True once the self test passed and the system was started. */
bool systemIsStarted(void);

/*
 * Arming. The drone starts disarmed; the motors only run while exactly one
 * pilot source holds the arm (or the parameter system.forceArm is set).
 * A second source cannot arm while another one holds it.
 */
typedef enum {
  ARM_SOURCE_NONE = 0,
  ARM_SOURCE_LEGACY,   // systemSetArmed()
  ARM_SOURCE_CRTP,     // mobile app, cfclient, ESP-NOW (CONFIG_CRTP_AUTO_ARM)
  ARM_SOURCE_WEB,      // browser control (webctrl)
} armSource_t;

/* Arm for this source. Returns false if another source holds the arm. */
bool systemRequestArm(armSource_t source);
/* Release the arm if this source holds it. */
void systemReleaseArm(armSource_t source);
/* Source currently holding the arm, ARM_SOURCE_NONE if disarmed. */
armSource_t systemGetArmOwner(void);
/* Emergency stop: disarm whoever holds the arm, clear system.forceArm. */
void systemForceDisarm(void);

#endif //__SYSTEM_H__
