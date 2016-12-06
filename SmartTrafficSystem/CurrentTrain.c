#include "header.h"
#define ITERATIONPERINDEX       2
#define XCOORDINATE1 0
#define XCOORDINATE3 42
#define XCOORDINATE4 30
#define YCOORDINATE0 2
#define YCOORDINATE1 12
#define YCOORDINATE2 22
#define YCOORDINATE3 32
#define YCOORDINATE4 42
#define YCOORDINATE5 52
#define YCOORDINATE6 62 // time countdown, time spent waiting
#define YCOORDINATE7 72
#define BRIGHTNESS 15
#define T2_HORIZONTAL_OFFSET 64
#define LONGWAITTHRESHOLD 36

#define TEN 10
#define HUNDRED 100
#define THOUSAND 1000
#define TENTHOUSAND 10000

extern xQueueHandle xOLEDQueue;
extern trainInfo trainQueue[5];
extern xTaskHandle xCurrentTrainHandle;

void vCurrentTrainTask(void *pvParameters)
{
  currentTrainD* mctd = (currentTrainD*) pvParameters;
  int soundPatternS[12] = {1, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0};
  int soundPatternN[10] = {1, 1, 0, 1, 1, 0, 1, 0, 1, 0};
  int soundPatternW[7] = {1, 1, 0, 1, 0, 1, 0};
  int soundPatternE[13] = {1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 0};
  while(1)
  {
    vTaskSuspend(xCurrentTrainHandle);
    if ((*mctd->trainCount > 0) && (*mctd->gridlock == FALSE) && (*mctd->trainCounter < *mctd->traversalCounter)) 
    {
   
      //---------------------------------------TRAIN 1--------------------------------------------
      
      // EAST
      if (*(trainQueue[0].toE)) 
      {
        // SOUND
        if (soundPatternE[(*mctd->trainCounter/ITERATIONPERINDEX) % (sizeof(soundPatternE)/sizeof(int))]) 
        {
          PWMGenEnable(PWM_BASE, PWM_GEN_0);
        } else {
          PWMGenDisable(PWM_BASE, PWM_GEN_0);
        }
        
        // FLASH
        if (*mctd->trainCounter % 8 < 4) { // flashing 2.0 sec
          printTrain(trainQueue[0], 0);
          printTrainOneStatus(*mctd->traversalCounter, *mctd->trainCounter);
          
        } else {
          clearDisplay(0);
        }
        
      // WEST  
      } 
      else if (*(trainQueue[0].toW)) 
      {
        
        // SOUND
        if (soundPatternW[(*mctd->trainCounter/ITERATIONPERINDEX) % (sizeof(soundPatternW)/sizeof(int))]) 
        {
          PWMGenEnable(PWM_BASE, PWM_GEN_0);
        } else {
          PWMGenDisable(PWM_BASE, PWM_GEN_0);
        }
        
        // FLASH
        if (*mctd->trainCounter % 4 < 2) 
        { // flashing 1.0 sec
          printTrain(trainQueue[0], 0);
          printTrainOneStatus(*mctd->traversalCounter, *mctd->trainCounter);
          
        } else {
          clearDisplay(0);
        }
      // SOUTH 
      } 
      else if (*(trainQueue[0].toS)) 
      {
        
        // SOUND
        if (soundPatternS[(*mctd->trainCounter/ITERATIONPERINDEX) % (sizeof(soundPatternS)/sizeof(int))]) 
        {
          PWMGenEnable(PWM_BASE, PWM_GEN_0);
        } else {
          PWMGenDisable(PWM_BASE, PWM_GEN_0);
        }     
        
        // FLASH
        if (*mctd->trainCounter % 4 < 2) 
        { // flashing 1.0 sec
          printTrain(trainQueue[0], 0);
          printTrainOneStatus(*mctd->traversalCounter, *mctd->trainCounter);
          
        } else {
          clearDisplay(0);
        }            
      
      // NORTH  
      } 
      else 
      {
        // SOUND
        if (soundPatternN[(*mctd->trainCounter/ITERATIONPERINDEX) % (sizeof(soundPatternN)/sizeof(int))]) 
        {
          PWMGenEnable(PWM_BASE, PWM_GEN_0);
        } else {
          PWMGenDisable(PWM_BASE, PWM_GEN_0);
        }
        
        // FLASH
        if (*mctd->trainCounter % 6 < 3) { // flashing 1.5 sec
          printTrain(trainQueue[0], 0);
          printTrainOneStatus(*mctd->traversalCounter, *mctd->trainCounter);
          
        } else {
          clearDisplay(0);
        }       
      }
      
  
    
      
      // ------------------------------------TRAIN 2-----------------------------------------
      
      if (*mctd->trainCount > 1)
      {
        printTrain(trainQueue[1], T2_HORIZONTAL_OFFSET);
        printTrainTwoStatus(*mctd->globalCount, *trainQueue[1].initializedTime);
      }
      else
      {
        clearDisplay(T2_HORIZONTAL_OFFSET);
      }
      (*mctd->trainCounter)++;
      
      
    }
    
  
      
    
  }
  
}

void printTrain(trainInfo train, int horizontalOffset)
{
  //--------------------------------to Direction--------------------------------
  if (*train.toN)
  {
    RIT128x96x4StringDraw("To:    N", XCOORDINATE1 + horizontalOffset, YCOORDINATE1, BRIGHTNESS);
  }
  else if (*train.toW)
  {
    RIT128x96x4StringDraw("To:    W", XCOORDINATE1 + horizontalOffset, YCOORDINATE1, BRIGHTNESS);
  }
  else if (*train.toE)
  {
    RIT128x96x4StringDraw("To:    E", XCOORDINATE1 + horizontalOffset, YCOORDINATE1, BRIGHTNESS);
  }
  else if (*train.toS)
  {
    RIT128x96x4StringDraw("To:    S", XCOORDINATE1 + horizontalOffset, YCOORDINATE1, BRIGHTNESS);
  }
  //--------------------------------from Direction------------------------------
  if (*train.fromN) 
  {
    RIT128x96x4StringDraw("From:  N", XCOORDINATE1 + horizontalOffset, YCOORDINATE2, BRIGHTNESS);
  } 
  else if (*train.fromW) 
  {
    RIT128x96x4StringDraw("From:  W", XCOORDINATE1 + horizontalOffset, YCOORDINATE2, BRIGHTNESS);
  } 
  else if (*train.fromE) 
  {
    RIT128x96x4StringDraw("From:  E", XCOORDINATE1 + horizontalOffset, YCOORDINATE2, BRIGHTNESS);
  } 
  else if (*train.fromS) 
  {
    RIT128x96x4StringDraw("From:  S", XCOORDINATE1 + horizontalOffset, YCOORDINATE2, BRIGHTNESS);
  }
  
  //--------------------------------passenger Count-----------------------------
  int passengerCount = *train.passengerCount;
  char ones = passengerCount % 10;
  char tens = (passengerCount % 100) / 10;
  char hundreds = (passengerCount % 1000) / 100;
  char num[] = {hundreds + '0', tens + '0', ones +'0', '\0'};
  RIT128x96x4StringDraw("PC: ", XCOORDINATE1 + horizontalOffset, YCOORDINATE3, BRIGHTNESS);
  RIT128x96x4StringDraw(num, XCOORDINATE4 + horizontalOffset, YCOORDINATE3, BRIGHTNESS);
  
  //-------------------------------train Size-----------------------------------
  char trainSizeChar[] = {*train.trainSize + '0', '\0'};
  RIT128x96x4StringDraw(trainSizeChar, XCOORDINATE3 + horizontalOffset, YCOORDINATE4, BRIGHTNESS);
  RIT128x96x4StringDraw("Size: ", XCOORDINATE1 + horizontalOffset, YCOORDINATE4, BRIGHTNESS);
  

  
  
}

void printTrainOneStatus(unsigned int traversalCounter, unsigned int trainCounter)
{
  //----------------------------Print Train 1 in Row 1-------------------------------
  RIT128x96x4StringDraw("Train 1", XCOORDINATE1, YCOORDINATE0, BRIGHTNESS);
  
  //----------------------------Print Time Left in Row 6-----------------------------
  RIT128x96x4StringDraw("TL: ", XCOORDINATE1, YCOORDINATE5, BRIGHTNESS);
  unsigned int timeLeft = ((traversalCounter - trainCounter) / 2);
  char ones = timeLeft % TEN;
  char tens = timeLeft / TEN;
  char timeLeftString[] = {tens + '0', ones +'0', '\0'};
  
  RIT128x96x4StringDraw(timeLeftString, XCOORDINATE1 + XCOORDINATE4, YCOORDINATE5, BRIGHTNESS );
  //---------------------------------------------------------------------------------
  
  // Print Choo Choo! in Row 7
  RIT128x96x4StringDraw("Choo Choo!", XCOORDINATE1, YCOORDINATE6, BRIGHTNESS);
}

void printTrainTwoStatus(unsigned int globalCount, unsigned int initializedTime)
{
  //----------------------------Print Train 2 in Row 1-------------------------------  
  RIT128x96x4StringDraw("Train 2", XCOORDINATE1 + T2_HORIZONTAL_OFFSET, YCOORDINATE0, BRIGHTNESS);
  
  //------------------------------Print Wait Time in Row 7---------------------------
  RIT128x96x4StringDraw("WT: ", XCOORDINATE1 + T2_HORIZONTAL_OFFSET, YCOORDINATE5, BRIGHTNESS);
  // print the time its been waiting for
  // (current globalcount - initialized global count) / 2 seconds
  unsigned int timeWaited = ((globalCount - initializedTime)/2);
  
  
  // TIME ELAPSED STRING 
  char ones = timeWaited % TEN;
  char tens = timeWaited / TEN;
  char hundreds = timeWaited / HUNDRED;
  char timeWaitedString[] = {hundreds + '0', tens + '0', ones +'0', '\0'};
  
  RIT128x96x4StringDraw(timeWaitedString, XCOORDINATE4 + T2_HORIZONTAL_OFFSET, YCOORDINATE5, BRIGHTNESS);
  //---------------------------------------------------------------------------------
  
  // Print Waiting... in Row 7
  RIT128x96x4StringDraw("Waiting...", XCOORDINATE1 + T2_HORIZONTAL_OFFSET, YCOORDINATE6, BRIGHTNESS);
  // Print Long wait... in row 8 if time waited is greater than the long wait threshold
  if (timeWaited >= LONGWAITTHRESHOLD)
  {
    RIT128x96x4StringDraw("Long wait!", XCOORDINATE1 + T2_HORIZONTAL_OFFSET, YCOORDINATE7, BRIGHTNESS);
  }
    
}

void clearDisplay(int horizontalOffset) 
{
  RIT128x96x4StringDraw("          ", XCOORDINATE1 + horizontalOffset, YCOORDINATE1, BRIGHTNESS);
  RIT128x96x4StringDraw("          ", XCOORDINATE1 + horizontalOffset, YCOORDINATE2, BRIGHTNESS);
  RIT128x96x4StringDraw("          ", XCOORDINATE1 + horizontalOffset, YCOORDINATE3, BRIGHTNESS);
  RIT128x96x4StringDraw("          ", XCOORDINATE1 + horizontalOffset, YCOORDINATE4, BRIGHTNESS);
  RIT128x96x4StringDraw("          ", XCOORDINATE1 + horizontalOffset, YCOORDINATE5, BRIGHTNESS);
  RIT128x96x4StringDraw("          ", XCOORDINATE1 + horizontalOffset, YCOORDINATE6, BRIGHTNESS);
  RIT128x96x4StringDraw("          ", XCOORDINATE1 + horizontalOffset, YCOORDINATE7, BRIGHTNESS);
}




