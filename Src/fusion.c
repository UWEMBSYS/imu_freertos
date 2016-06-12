#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "imu.h"
#include "fusion.h"

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
    /* TODO: Add fusion */
    /* repackage data */
    
    /* Pass it onto the display task */
    ret = xQueueSend(qFusionToDisplay, &fusedData, 110);
    if (pdTRUE != ret) {
      printf("Fusion did not post data\r\n");
      continue;
    }
  }
}
