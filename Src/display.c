#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"

#include "display.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "imu.h"
#include "fusion.h"
#include "MadgwickAHRS.h"

FusedData_t fusedData;
void DisplayTaskThread(void * param)
{
  BaseType_t ret;

  printf("Display task starting\r\n");
  unsigned count = 0;
  while (1) {
     /* Wait for data to arrive */
    ret = xQueueReceive(qFusionToDisplay, &fusedData, portMAX_DELAY );
    
    if (pdTRUE != ret) {
      printf("Display did not get data\r\n");
      continue;
    }
    if (count++ % 10 == 0) {
      /* output the data */
      printf("Time: %lu\r\n", fusedData.raw.time );
      //printf("AccXYZ: %d %d %d\r\n", data.raw.acc.x, data.raw.acc.y, data.raw.acc.z);
      //printf("GyroXYZ: %d %d %d\r\n", data.raw.gyro.x, data.raw.gyro.y, data.raw.gyro.z);
      //printf("magXYZ:  %d %d %d\r\n",  data.raw.mag.x, data.raw.mag.y, data.raw.mag.z);
      printf("quat: %f %f %f %f\r\n", fusedData.q.w, fusedData.q.x, fusedData.q.y,fusedData.q.z);
      MadgwickComputeAngles();
      printf("PRY:%f:%f:%f\r\n", pitch,roll,yaw);
    }

  }
}

