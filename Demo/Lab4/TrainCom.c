#include "header.h"

extern xSemaphoreHandle xSemaphore;

extern trainInfo trainQueue[5];
extern xTaskHandle xTrainComHandle;
void vTrainComTask(void *pvParameters) {
  trainComD* mtcd = (trainComD*) pvParameters;
  while (1) 
  {
    vTaskSuspend(xTrainComHandle);

    int rand0_3 = rng(0, 3);
    int rand2_9 = rng(2, 9);
    int numTrain = *mtcd->trainCount;
    
    // Add train to the queue unless there's already 5 trains in the queue
    if (numTrain < 5) {
      
      if (rand0_3 == 0) // toEast
      {
        *trainQueue[numTrain].toE = TRUE;
      } else if (rand0_3 == 1) // toWest
      {
        *trainQueue[numTrain].toW = TRUE;
      } else if (rand0_3 == 2) // toSouth
      {
        *trainQueue[numTrain].toS = TRUE;
      } else if (rand0_3 == 3)// toNorth
      {
        *trainQueue[numTrain].toN = TRUE;
      }
      
      // fromDirection
      if (*mtcd->fromState == 1) 
      {
        *trainQueue[numTrain].fromN = TRUE;
      } else if (*mtcd->fromState == 2)
      {
        *trainQueue[numTrain].fromS = TRUE;
      } else if (*mtcd->fromState == 4) 
      {
        *trainQueue[numTrain].fromW = TRUE;
      } else if (*mtcd->fromState == 8)
      {  
        *trainQueue[numTrain].fromE = TRUE;
      }

      // reset fromState
      *mtcd->fromState = 0;
      
      // used for debugging
      // *(trainQueue[numTrain].trainSize) = 7;
      *(trainQueue[numTrain].trainSize) = rng(2, 9);

      // passenger count
      *(trainQueue[numTrain].passengerCount) = *mtcd->passengerCount;
      if (*(trainQueue[numTrain].passengerCount) < 0) { 
        *(trainQueue[numTrain].passengerCount)= 0; 
      } else if (*(trainQueue[numTrain].passengerCount) > 300) { 
        *(trainQueue[numTrain].passengerCount) = 300; 
      } 
      
      // increment train count
      *mtcd->trainCount = numTrain + 1;
      
      // set the time that the train was intialized in terms of GC
      *trainQueue[numTrain].initializedTime = *mtcd->globalCount;
    }
    
    // to prevent double button presses on a single button press
    vTaskDelay(250); 
  }
}    
    

