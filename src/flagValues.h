#ifndef FLAG_VALUES_H
#define FLAG_VALUES_H

#include <stdio.h>

/*
    List od ble read commands
*/

uint8_t startStream_cmd[10];
uint8_t stopStream_cmd[10];
uint8_t reset_cmd[10];
uint8_t bmxIMU_cmd[10];
uint8_t icmIMU_cmd[10];

bool startStopStream_flag = true;
bool reset_flag = false;
bool bmxIMU_flag = false;
bool icmIMU_flag = true;

void initCmds()
{
    //Start/stop command
    sprintf(startStream_cmd, "start");
    sprintf(stopStream_cmd, "stop");
    //Reset command
    sprintf(reset_cmd, "reset");
    //Send BMX160 IMU data  command
    sprintf(bmxIMU_cmd, "bmx");
    //Send ICM IMU data command
    sprintf(icmIMU_cmd, "icm");
}

void checkCmd(const uint8_t *const data, uint16_t len)
{
  //Start cmd:
  if(memcmp(&startStream_cmd, data, len) == 0)
  {
    startStopStream_flag = true;
    reset_flag = false;
    bmxIMU_flag = false;
    icmIMU_flag = false;
    return;
  }
  //Stop cmd:
  if(memcmp(&stopStream_cmd, data, len) == 0)
  {
    startStopStream_flag = false;
    reset_flag = false;
    bmxIMU_flag = false;
    icmIMU_flag = false;
    return;
  }
  //Reset cmd:
  if(memcmp(&reset_cmd, data, len) == 0)
  {
    reset_flag = true;
    bmxIMU_flag = false;
    icmIMU_flag = false;
    return;
  }
  //BMX160 cmd:
  if(memcmp(&bmxIMU_cmd, data, len) == 0)
  {
    bmxIMU_flag = true;
    icmIMU_flag = false;
    reset_flag = false;
    return;
  }
  //ICM42688 cmd:
  if(memcmp(&icmIMU_cmd, data, len) == 0)
  {
    icmIMU_flag = true;
    bmxIMU_flag = false;
    reset_flag = false;
    return;
  }
}
#endif /* FLAG_VALUES_H */
