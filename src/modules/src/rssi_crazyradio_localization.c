#include <string.h>
#include <stdint.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "log.h"
#include "system.h"


#include <rssi_crazyradio_localization.h>
#include <radiolink.h>

uint8_t rssi;
static bool isInit;
static float distance;
float gamma_rrsi = 1.8f;
float Pn = 45.0f;
void rssiCrazyradioLocalizationInit(void)
{
  if (isInit)
    return;
  xTaskCreate(rssiCrazyRadioLocalizationTask,"RSSI_CRAZYRADIO_LOCALIZATION",ZRANGER_TASK_STACKSIZE, NULL,ZRANGER_TASK_PRI,NULL );
  isInit = true;

}

void rssiCrazyRadioLocalizationTask(void* arg)
{
  systemWaitStart();

  while(1) {
    vTaskDelay(10);
    float temp = (-50.0f+(float)rssi)/(10*gamma_rrsi);
    distance = 100*pow(10,temp);
  }



}

LOG_GROUP_START(rssiCrazyradio)
LOG_ADD(LOG_UINT8, rssi, &rssi)
LOG_ADD(LOG_FLOAT, distance, &distance)
LOG_GROUP_STOP(rssiCrazyradio)
