#include "header.h"
#define GRIDLOCKCONSTANT -24 //0.2 * 60 sec * 2 delay_counts/sec
#define TRAVERSALCONSTANT 12
#define XCOORDINATE1 0
#define YCOORDINATE1 24
#define YCOORDINATE2 40
#define YCOORDINATE3 56
#define BRIGHTNESS 15
extern trainInfo trainQueue[5];
extern xTaskHandle xSwitchControlHandle;
extern xQueueHandle xOLEDQueue;

void vSwitchControlTask(void *pvParameters) {
  switchControlD* mscd = (switchControlD*) pvParameters;
  
  while(1) 
  {
    vTaskSuspend(xSwitchControlHandle);
    // if train is present
    if (*mscd->trainCount > 0) 
    {
          
      // no train in transit
            if (*mscd->trainCounter == 0) 
      {
        *mscd->traversalCounter = *(trainQueue[0].trainSize) * TRAVERSALCONSTANT;
      }
      

      
      
      else if (*mscd->trainCounter == *mscd->traversalCounter) {
        
        // reset global
        *mscd->trainCounter = 0;
        *mscd->traversalCounter = 0;
        (*mscd->trainCount)--;
        
        
        // move over trains in queue
        for (int i = 0; i < 4; i++) {
          *(trainQueue[i].toE) = *(trainQueue[i+1].toE);
          *(trainQueue[i].toW) = *(trainQueue[i+1].toW);
          *(trainQueue[i].toS) = *(trainQueue[i+1].toS);
          *(trainQueue[i].toN) = *(trainQueue[i+1].toN);
          *(trainQueue[i].fromE) = *(trainQueue[i+1].fromE);
          *(trainQueue[i].fromW) = *(trainQueue[i+1].fromW);
          *(trainQueue[i].fromS) = *(trainQueue[i+1].fromS);
          *(trainQueue[i].fromN) = *(trainQueue[i+1].fromN);
          *(trainQueue[i].passengerCount) = *(trainQueue[i+1].passengerCount);
          *(trainQueue[i].trainSize) = *(trainQueue[i+1].trainSize);
          *(trainQueue[i].initializedTime) = *(trainQueue[i+1].initializedTime);
        }
        *(trainQueue[4].toE) = FALSE;
        *(trainQueue[4].toW) = FALSE;
        *(trainQueue[4].toS) = FALSE;
        *(trainQueue[4].toN) = FALSE;
        *(trainQueue[4].fromE) = FALSE;
        *(trainQueue[4].fromW) = FALSE;
        *(trainQueue[4].fromS) = FALSE;
        *(trainQueue[4].fromN) = FALSE;
        *(trainQueue[4].passengerCount) = 0;
        *(trainQueue[4].trainSize) = 0;
        *(trainQueue[4].initializedTime) = 0;
      }
    }
    
    
    
  }
}


//      // no train in transit and done with gridlock at the moment
//      if (*mscd->gridlockDelay == 0 && *mscd->trainCounter == 0) 
//      {
//        
//        *mscd->gridlock = FALSE;
//        int gridlockRand = rng(-2, 2);
//        
//        if (gridlockRand < 0) 
//        {
//          *mscd->gridlock = TRUE;
//          *mscd->gridlockDelay = GRIDLOCKCONSTANT * gridlockRand;
//        } 
//        else 
//        {
//          *mscd->traversalCounter = *(trainQueue[0].trainSize) * TRAVERSALCONSTANT;
//        }
//      } 
//      
//      // if gridlock is on
//      else if (*mscd->gridlock) 
//      {
//        (*mscd->gridlockDelay)--;
//        
//        // display gridlock
//        if ((*mscd->gridlockDelay % 2) == 0) 
//        {
//          RIT128x96x4StringDraw("Gridlock!!!", XCOORDINATE1, YCOORDINATE1, BRIGHTNESS);
//        } 
//        // clear screen
//        else 
//        {
//          RIT128x96x4StringDraw("                     ", XCOORDINATE1, YCOORDINATE1, BRIGHTNESS);
//          RIT128x96x4StringDraw("                     ", XCOORDINATE1, YCOORDINATE2, BRIGHTNESS);
//          RIT128x96x4StringDraw("                     ", XCOORDINATE1, YCOORDINATE3, BRIGHTNESS);
//          RIT128x96x4StringDraw("                     ", XCOORDINATE1, YCOORDINATE1, BRIGHTNESS);
//        }
//      } 

//
//      // if gridlock is on
//      else if (*mscd->gridlock) 
//      {
//        (*mscd->gridlockDelay)--;
//        
//        // display gridlock
//        if ((*mscd->gridlockDelay % 2) == 0) 
//        {
//          RIT128x96x4StringDraw("Gridlock!!!", XCOORDINATE1, YCOORDINATE1, BRIGHTNESS);
//        } 
//        // clear screen
//        else 
//        {
//          RIT128x96x4StringDraw("                     ", XCOORDINATE1, YCOORDINATE1, BRIGHTNESS);
//          RIT128x96x4StringDraw("                     ", XCOORDINATE1, YCOORDINATE2, BRIGHTNESS);
//          RIT128x96x4StringDraw("                     ", XCOORDINATE1, YCOORDINATE3, BRIGHTNESS);
//          RIT128x96x4StringDraw("                     ", XCOORDINATE1, YCOORDINATE1, BRIGHTNESS);
//        }
//      } 