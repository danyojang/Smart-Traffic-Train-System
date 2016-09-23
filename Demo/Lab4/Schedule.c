#include "header.h"
#define XCOORDINATE1 0
#define XCOORDINATE2 100
#define YCOORDINATE0 0
#define YCOORDINATE1 24
#define YCOORDINATE2 40
#define YCOORDINATE3 56
#define YCOORDINATE4 70
#define XCOORDINATEGC 85
#define YCOORDINATEGC 85
#define BRIGHTNESS 15
#define TEN 10
#define HUNDRED 100
#define THOUSAND 1000
#define TENTHOUSAND 10000

extern xSemaphoreHandle xSemaphore;
extern xQueueHandle xOLEDQueue;
extern xTaskHandle xTrainComHandle;
extern xTaskHandle xCurrentTrainHandle;
extern xTaskHandle xSwitchControlHandle;
extern xTaskHandle xSerialComHandle;
void vScheduleTask(void* data)
{
  scheduleD* msd = (scheduleD*) data;
  
  while(1)
  {
    char ones = *msd->globalCount % TEN;
    char tens = (*msd->globalCount % HUNDRED) / TEN;
    char hundreds = (*msd->globalCount % THOUSAND) / HUNDRED;
    char thousands = (*msd->globalCount % TENTHOUSAND) / THOUSAND;
    char gc[] = {thousands + '0', hundreds + '0', tens + '0', ones +'0', '\0'};
    RIT128x96x4StringDraw("Global Count: ", XCOORDINATE1, YCOORDINATEGC, BRIGHTNESS);
    RIT128x96x4StringDraw(gc, XCOORDINATEGC, YCOORDINATEGC, BRIGHTNESS);
    if (*msd->trainCount > 0) {
      vTaskResume(xSwitchControlHandle);
      vTaskResume(xCurrentTrainHandle);
    }
    vTaskResume(xSerialComHandle);
    // reset frequency count every half second
    *msd->passengerCount = ((*msd->freqCount) * 6 / 10) - 300;
    *msd->freqCount = 0;
    
    *msd->globalCount = *msd->globalCount + 1;
    if (*msd->trainCount == 0)
    {
      PWMGenDisable(PWM_BASE, PWM_GEN_0);
    }

    delay();
  }
}

// delays task for 500ms and makes random call more random
void delay() {
    int i;
    
    for (i = 0; i < 5; i++) {
        rand();
        vTaskDelay(100);
    }
}