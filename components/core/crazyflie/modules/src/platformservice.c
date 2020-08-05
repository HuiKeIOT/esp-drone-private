/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
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
 * crtpservice.c - Implements low level services for CRTP
 */

#include <stdbool.h>

/* FreeRtos includes */
#include "freertos/FreeRTOS.h"
#include <stdint.h>
#include <string.h>

#include "config.h"
#include "crtp.h"
#include "platformservice.h"
#include "version.h"
#include "platform.h"

static bool isInit=false;

typedef enum {
  platformCommand   = 0x00,
  versionCommand    = 0x01,
} Channel;

typedef enum {
  setContinousWave   = 0x00,
} PlatformCommand;

typedef enum {
  getProtocolVersion = 0x00,
  getFirmwareVersion = 0x01,
  getDeviceTypeName  = 0x02,
} VersionCommand;

void platformserviceHandler(CRTPPacket *p);
static void platformCommandProcess(uint8_t command, uint8_t *data);
static void versionCommandProcess(CRTPPacket *p);

void platformserviceInit(void)
{
  if (isInit)
    return;

  // Register a callback to service the Platform port
  crtpRegisterPortCB(CRTP_PORT_PLATFORM, platformserviceHandler);

  isInit = true;
}

bool platformserviceTest(void)
{
  return isInit;
}

void platformserviceHandler(CRTPPacket *p)
{
  switch (p->channel)
  {
    case platformCommand:
      platformCommandProcess(p->data[0], &p->data[1]);
      crtpSendPacket(p);
      break;
    case versionCommand:
      versionCommandProcess(p);
    default:
      break;
  }
}

static void platformCommandProcess(uint8_t command, uint8_t *data)
{
    switch (command) {
        case setContinousWave:
            //TODO:
            break;

        default:
            break;
    }
}

static void versionCommandProcess(CRTPPacket *p)
{
  switch (p->data[0]) {
    case getProtocolVersion:
      *(int*)&p->data[1] = PROTOCOL_VERSION;
      p->size = 5;
      crtpSendPacket(p);
      break;
    case getFirmwareVersion:
      strncpy((char*)&p->data[1], V_STAG, CRTP_MAX_DATA_SIZE-1);
      p->size = (strlen(V_STAG)>CRTP_MAX_DATA_SIZE-1)?CRTP_MAX_DATA_SIZE:strlen(V_STAG)+1;
      crtpSendPacket(p);
      break;
    case getDeviceTypeName:
      {
      const char* name = platformConfigGetDeviceTypeName();
      strncpy((char*)&p->data[1], name, CRTP_MAX_DATA_SIZE-1);
      p->size = (strlen(name)>CRTP_MAX_DATA_SIZE-1)?CRTP_MAX_DATA_SIZE:strlen(name)+1;
      crtpSendPacket(p);
      }
      break;
    default:
      break;
  }
}
