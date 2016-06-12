#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"

#include "display.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "imu.h"
#include "fusion.h"

void DisplayTaskThread(void * param)
{
  BaseType_t ret;
  FusedData_t data;
  printf("Display task starting\r\n");
  while (1) {
     /* Wait for data to arrive */
    ret = xQueueReceive(qFusionToDisplay, &data, portMAX_DELAY );
    
    if (pdTRUE != ret) {
      printf("Display did not get data\r\n");
      continue;
    }
      
    /* output the data */
    printf("Time: %u\r\n", data.raw.time );
    printf("AccXYZ: %x %x %x\r\n", data.raw.acc.x, data.raw.acc.y, data.raw.acc.z);
    printf("GyroXYZ: %x %x %x\r\n", data.raw.gyro.x, data.raw.gyro.y, data.raw.gyro.z);
    printf("magXYZ:  %x %x %x\r\n",  data.raw.mag.x, data.raw.mag.y, data.raw.mag.z);
    printf("quat: %f %f %f %f\r\n", data.q.w, data.q.x, data.q.y,data.q.z);

  }
}

