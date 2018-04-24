#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "system.h"


#include <rssi_crazyradio_localization.h>
#include <radiolink.h>

uint8_t rssi;
static bool isInit;


void rssiCrazyradioLocalizationInit(void)
{
/*  if (isInit)
    return;
  xTaskCreate(rssiCrazyRadioLocalizationTask,"RSSI_CRAZYRADIO_LOCALIZATION",ZRANGER_TASK_STACKSIZE, NULL,ZRANGER_TASK_PRI,NULL );
  isInit = true;*/

}

void rssiCrazyRadioLocalizationTask(void* arg)
{
/*
  systemWaitStart();

  while(1) {
    vTaskDelay(10);
  }
*/



}

/*LOG_GROUP_START(rssi_crazyradio_localization)
LOG_ADD(LOG_UINT8, rssi, &rssi)
LOG_GROUP_STOP(rssi_crazyradio_localization)*/
