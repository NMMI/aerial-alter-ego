
// Copyright (c) 2016, Mattia Poggiani.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 *  \file       imuboard_communications.cpp
 *
 *  \brief      Library of functions for serial port communication with a
 *              IMU board
 *
 *  \details
 *
 *  Check the \ref imuboard_communications.h "imuboard_communications.h" file
 *  for a complete description of the public functions implemented in
 *  imuboard_communications.cpp.
 **/

//=================================================================     includes

#include <stdio.h>  /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <stdint.h>
#include <ctype.h>
#include <time.h>

#if (defined(_WIN32) || defined(_WIN64))
#include <windows.h>
#endif

#if !(defined(_WIN32) || defined(_WIN64))
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/ioctl.h>
#include <dirent.h>
#include <sys/time.h>
#include <stdlib.h>
#endif

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__))
#include <linux/serial.h>
#endif

#if (defined(__APPLE__))
#include <IOKit/IOKitLib.h>
#include <IOKit/serial/IOSerialKeys.h>
#include <IOKit/serial/ioss.h>
#include <IOKit/IOBSD.h>
#endif

#include "imuboard_communications.h"
#include "imuboard_commands.h"

#define BUFFER_SIZE 500 ///< Size of buffers that store communication packets

//===========================================     public fuctions implementation

/// @cond C_FILES

//==============================================================================
//   	                                                      commGetImuReadings
//==============================================================================
// Retrieve accelerometers, gyroscopes and magnetometers readings.
//==============================================================================

void commGetImuReadings(comm_settings *comm_settings_t, int id, uint8_t *imu_table, uint8_t *imus_magcal, int n_imu, float *imu_values)
{

  char data_out[BUFFER_SIZE];   // output data buffer
  char package_in[BUFFER_SIZE]; // output data buffer
  int package_in_size;
  float acc_sf = 0, gyro_sf = 0, mag_sf = 0;
  float temp_sf = 0, temp_off = 0, temp_div = 0;
  char *values;
  int c = 0;
  float aux_float[3];
  int16_t aux_si;

#if (defined(_WIN32) || defined(_WIN64))
  DWORD package_size_out; // for serial port access
#else
  int n_bytes;
#endif

  //=================================================		preparing packet to send

  data_out[0] = ':';
  data_out[1] = ':';
  data_out[2] = (unsigned char)id;
  data_out[3] = 2;
  data_out[4] = CMD_GET_IMU_READINGS; // command
  data_out[5] = CMD_GET_IMU_READINGS; // checksum

#if (defined(_WIN32) || defined(_WIN64))
  WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
#else
  ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
  if (n_bytes)
    read(comm_settings_t->file_handle, package_in, n_bytes);

  write(comm_settings_t->file_handle, data_out, 6);
#endif

  memset(package_in, 0, sizeof(package_in));

  package_in_size = RS485read(comm_settings_t, id, package_in);
  if (package_in_size == -1)
    return;

  // acc_sf 	= 0.000061037 * 2;			// Ticks to G
  acc_sf = 0.000061037; // Ticks to G
  // gyro_sf = 0.007629627 * 8;		// Ticks to deg/s with FS +/- 2000 °/s
  gyro_sf = 0.007629627 * 8; // Ticks to deg/s with FS +/- 2000 °/s
  mag_sf = 0.1465;           // Ticks to uT

  temp_sf = 0.00294118; // 1/340 //0.001426;
  temp_off = 36.53;     // 21.6;
  temp_div = 2.0;

  values = &package_in[1];

  for (int i = 0; i < n_imu; i++)
  {

    if (values[c] == ':')
    {

      if (imu_table[5 * i + 0])
      {
        ((char *)&aux_si)[0] = values[c + 2];
        ((char *)&aux_si)[1] = values[c + 1];
        aux_float[0] = (float)(aux_si * acc_sf);

        ((char *)&aux_si)[0] = values[c + 4];
        ((char *)&aux_si)[1] = values[c + 3];
        aux_float[1] = (float)(aux_si * acc_sf);
        ((char *)&aux_si)[0] = values[c + 6];
        ((char *)&aux_si)[1] = values[c + 5];
        aux_float[2] = (float)(aux_si * acc_sf);

        imu_values[(3 * 3 + 4 + 1) * i] = aux_float[0];
        imu_values[(3 * 3 + 4 + 1) * i + 1] = aux_float[1];
        imu_values[(3 * 3 + 4 + 1) * i + 2] = aux_float[2];
        c += 6;
      }
      if (imu_table[5 * i + 1])
      {
        ((char *)&aux_si)[0] = values[c + 2];
        ((char *)&aux_si)[1] = values[c + 1];
        aux_float[0] = (float)(aux_si * gyro_sf);
        ((char *)&aux_si)[0] = values[c + 4];
        ((char *)&aux_si)[1] = values[c + 3];
        aux_float[1] = (float)(aux_si * gyro_sf);
        ((char *)&aux_si)[0] = values[c + 6];
        ((char *)&aux_si)[1] = values[c + 5];
        aux_float[2] = (float)(aux_si * gyro_sf);

        imu_values[(3 * 3 + 4 + 1) * i + 3] = aux_float[0];
        imu_values[(3 * 3 + 4 + 1) * i + 4] = aux_float[1];
        imu_values[(3 * 3 + 4 + 1) * i + 5] = aux_float[2];
        c += 6;
      }
      if (imu_table[5 * i + 2])
      {
        ((char *)&aux_si)[0] = values[c + 2];
        ((char *)&aux_si)[1] = values[c + 1];
        aux_float[0] = (float)(aux_si * mag_sf * (float)imus_magcal[3 * i + 0]);
        ((char *)&aux_si)[0] = values[c + 4];
        ((char *)&aux_si)[1] = values[c + 3];
        aux_float[1] = (float)(aux_si * mag_sf * (float)imus_magcal[3 * i + 1]);
        ((char *)&aux_si)[0] = values[c + 6];
        ((char *)&aux_si)[1] = values[c + 5];
        aux_float[2] = (float)(aux_si * mag_sf * (float)imus_magcal[3 * i + 2]);

        imu_values[(3 * 3 + 4 + 1) * i + 6] = -aux_float[1];
        imu_values[(3 * 3 + 4 + 1) * i + 7] = -aux_float[0];
        imu_values[(3 * 3 + 4 + 1) * i + 8] = aux_float[2];
        c += 6;
      }
      if (imu_table[5 * i + 3])
      {
        ((char *)&aux_si)[0] = values[c + 2];
        ((char *)&aux_si)[1] = values[c + 1];
        aux_float[0] = (float)(aux_si);
        ((char *)&aux_si)[0] = values[c + 4];
        ((char *)&aux_si)[1] = values[c + 3];
        aux_float[1] = (float)(aux_si);
        ((char *)&aux_si)[0] = values[c + 6];
        ((char *)&aux_si)[1] = values[c + 5];
        aux_float[2] = (float)(aux_si);
        ((char *)&aux_si)[0] = values[c + 8];
        ((char *)&aux_si)[1] = values[c + 7];
        aux_float[3] = (float)(aux_si);

        imu_values[(3 * 3 + 4 + 1) * i + 9] = aux_float[0];
        imu_values[(3 * 3 + 4 + 1) * i + 10] = aux_float[1];
        imu_values[(3 * 3 + 4 + 1) * i + 11] = aux_float[2];
        imu_values[(3 * 3 + 4 + 1) * i + 12] = aux_float[3];
        c += 8;
      }
      if (imu_table[5 * i + 4])
      {
        ((char *)&aux_si)[0] = values[c + 2];
        ((char *)&aux_si)[1] = values[c + 1];
        aux_float[0] = (float)(aux_si * (float)temp_sf + temp_off) / temp_div;

        imu_values[(3 * 3 + 4 + 1) * i + 13] = aux_float[0];
        c += 2;
      }

      // printf("\n");
      c = c + 1;
    }
    if (values[c] == ':')
      c = c + 1;
    else
    {
      break;
      // printf("Break at %d\n", c);
    }
  }

  // usleep(8000);
}

/// @endcond

/* [] END OF FILE */
