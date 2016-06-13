#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "imu.h"
#include "fusion.h"
#include "MadgwickAHRS.h"

void FusionTaskThread(void * param)
{

    /* setup fusion */
  printf("Fusion task starting\r\n");
  BaseType_t ret;
  FusedData_t fusedData;
  
  while (1) {
     /* Wait for data to arrived */
    ret = xQueueReceive(qImuToFusion, &fusedData.raw, portMAX_DELAY );
    
    if (pdTRUE != ret) {
      printf("Fusion did not get data\r\n");
      continue;
    }
      
    /* Process the data */
    float gx, gy, gz, ax, ay, az, mx, my, mz;
    
    /* convert units, gyro wants rads/second. Range is +/- 2000 dps */
    gx = fusedData.raw.gyro.x * ((8.75f / 1000.0f ) * 0.01745329f ) ;
    gy = fusedData.raw.gyro.y * ((8.75f / 1000.0f ) * 0.01745329f ) ;
    gz = fusedData.raw.gyro.z * ((8.75f / 1000.0f ) * 0.01745329f ) ;
    
    /* convert to g */
    ax = fusedData.raw.acc.x * 0.061f ;
    ay = fusedData.raw.acc.y * 0.061f ;
    az = fusedData.raw.acc.z * 0.061f ;
    
    /* In gauss already, scaled +/- 4Gauss */
    mx =  fusedData.raw.mag.x * 6842.0f;
    my =  fusedData.raw.mag.y * 6842.0f;
    mz =  fusedData.raw.mag.z * 6842.0f;
    
    MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz); 

    /* repackage data */
    fusedData.q.w = q0;
    fusedData.q.x = q1;
    fusedData.q.y = q2;
    fusedData.q.z = q3;
    
    /* Pass it onto the display task */
    ret = xQueueSend(qFusionToDisplay, &fusedData, 110);
    if (pdTRUE != ret) {
      printf("Fusion did not post data\r\n");
      continue;
    }
  }
}
