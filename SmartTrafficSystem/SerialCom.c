#include "header.h"
#define CLEARSCREEN 10
#define LONGWAITTHRESHOLD 36
#define TEN 10
#define HUNDRED 100
#define THOUSAND 1000
#define TENTHOUSAND 10000
extern trainInfo trainQueue[5];
extern xTaskHandle xSerialComHandle;
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount) {
  //
  // Loop while there are more characters to send.
  //
  while(ulCount--)
  {
    //
    // Write the next character to the UART.
    //
    UARTCharPut(UART0_BASE, *pucBuffer++);
  }
}

void vSerialComTask(void* data) {
  serialComD* scdp = (serialComD*) data;
  while (1)
  {
    vTaskSuspend(xSerialComHandle);
    
    // clear screen
    UARTSend((unsigned char *)"\033c", 5);
    
    // ------------------PRINT TRAIN LABELS----------------------
    printUART("Train1 \t\t\t Train2");
    newLine();
    
    
    // --------------------CLEAR SCREEN--------------------------
    if (*scdp->trainCount == 0)
    {
      int i = 0;
      for (i = 0; i < CLEARSCREEN; i++)
      {
        newLine();
      }
    }
    
    
    else if (*scdp->trainCount  > 0)
    {
      
      // --------------PRINT TRAIN0 TO DIRECTION----------------
      if (*trainQueue[0].toN)
      {
        printUART("To: N");
      }
      else if (*trainQueue[0].toW)
      {
        printUART("To: W");
      }
      else if (*trainQueue[0].toE)
      {
        printUART("To: E");
      }
      else if (*trainQueue[0].toS)         
      {
        printUART("To: S");
      }
      
      // SEPERATE BY COLUMN
      printUART("                    ");
      
      // --------------PRINT TRAIN1 TO DIRECTION----------------
      if (*scdp->trainCount > 1)
      {
        if (*trainQueue[1].toN)
        {
          printUART("To: N");
        }
        else if (*trainQueue[1].toW)
        {
          printUART("To: W");
        }
        else if (*trainQueue[1].toE)
        {
          printUART("To: E");
        }
        else if (*trainQueue[1].toS)         
        {
          printUART("To: S");
        }
      }
      
      newLine();
      
      // --------------PRINT TRAIN0 FROM DIRECTION----------------
      if (*trainQueue[0].fromN)
      {
        printUART("From: N");
      }
      else if (*trainQueue[0].fromW)
      {
        printUART("From: W");
      }
      else if (*trainQueue[0].fromE)
      {
        printUART("From: E");
      }
      else if (*trainQueue[0].fromS)
      {
        printUART("From: S");
      }
      
      
      printUART("                  ");
      
      
      // --------------PRINT TRAIN1 FROM DIRECTION----------------
      if (*scdp->trainCount > 1)
      {
        if (*trainQueue[1].fromN)
        {
          printUART("From: N");
        }
        else if (*trainQueue[1].fromW)
        {
          printUART("From: W");
        }
        else if (*trainQueue[1].fromE)
        {
          printUART("From: E");
        }
        else if (*trainQueue[1].fromS)
        {
          printUART("From: S");
        }
      }
      
      newLine();
      
      // ---------------PRINT TRAIN0 PASSENGER COUNT----------------
      int passengerCount0 = *trainQueue[0].passengerCount;
      char onesPC0 = passengerCount0 % 10;
      char tensPC0 = (passengerCount0 % 100) / 10;
      char hundredsPC0 = (passengerCount0 % 1000) / 100;
      char pc0[] = {hundredsPC0 + '0', tensPC0 + '0', onesPC0 +'0', '\0'};
      printUART("Passenger Count: ");
      printUART(pc0);
      
      
      printUART("     ");
      
      
      // ---------------PRINT TRAIN1 PASSENGER COUNT---------------
      if (*scdp->trainCount > 1)
      {
      
        int passengerCount1 = *trainQueue[1].passengerCount;
        char onesPC1 = passengerCount1 % 10;
        char tensPC1 = (passengerCount1 % 100) / 10;
        char hundredsPC1 = (passengerCount1 % 1000) / 100;
        char pc1[] = {hundredsPC1 + '0', tensPC1 + '0', onesPC1 +'0', '\0'};
        printUART("Passenger Count: ");
        printUART(pc1);
      }
      newLine();
      
      // -----------------PRINT TRAIN0 SIZE-----------------------
      char train1SizeChar[] = {*trainQueue[0].trainSize + '0', '\0'};
      printUART("Train Size: ");
      printUART(train1SizeChar);
      
      printUART("            ");
      
      
      // -----------------PRINT TRAIN1 SIZE-----------------------
      if (*scdp->trainCount > 1)
      {
        char train2SizeChar[] = {*trainQueue[0].trainSize + '0', '\0'};
        printUART("Train Size: ");
        printUART(train2SizeChar);
      }
      
      newLine();
      
      // -----------------------PRINT TRAIN0 TIME LEFT-----------------------
      unsigned int timeLeft = ((*scdp->traversalCounter - *scdp->trainCounter) / 2);
      char onesTL = timeLeft % TEN;
      char tensTL = timeLeft / TEN;
      char timeLeftString[] = {tensTL + '0', onesTL +'0', '\0'};
      printUART("Time Left: ");
      printUART(timeLeftString);
      
      printUART("            ");
      
      
      // ----------------------PRINT TRAIN1 TIME WAITED-----------------------
      if (*scdp->trainCount > 1)
      {
        // print the time its been waiting for
        // (current globalcount - initialized global count) / 2 seconds
        unsigned int timeWaited = ((*scdp->globalCount - *trainQueue[1].initializedTime)/2);

        // TIME ELAPSED STRING 
        char onesTW = timeWaited % TEN;
        char tensTW = timeWaited / TEN;
        char hundredsTW = timeWaited / HUNDRED;
        char timeWaitedString[] = {hundredsTW + '0', tensTW + '0', onesTW +'0', '\0'};
        printUART("Time Waited: ");
        printUART(timeWaitedString);
      }
      
      
      newLine();
      
      
      // --------------------PRINT CHOO CHOO!------------------------------
      if (*scdp->trainCount > 0)
      {
        printUART("Choo Choo!");
      }
      
      printUART("               ");
      
      
      // ----------------------PRINT WAITING------------------------------
      if (*scdp->trainCount > 1)
      {
        printUART("Waiting...");
      }
      newLine();
      
      // ----------------------PRINT BREAK TEMPERATURE -----------------------
      
      printUART("                   ");
      
      // -----------------------------PRINT LONG WAIT------------------------
      if (*scdp->trainCount > 1)
      {
        
        // timeWaited is 6 or more car lengths (36 seconds)
        if (((*scdp->globalCount - *trainQueue[1].initializedTime)/2) >= 36)
        {
          printUART("Long wait!");
        }
      }
 
      newLine();

      
    }
   
    newLine();
    
    //--------------------------GLOBAL COUNT----------------------------
    char ones = *scdp->globalCount % TEN;
    char tens = (*scdp->globalCount % TEN) / TEN;
    char hundreds = (*scdp->globalCount % THOUSAND) / HUNDRED;
    char thousands = (*scdp->globalCount % TENTHOUSAND) / THOUSAND;
    char gc[] = {thousands + '0', hundreds + '0', tens + '0', ones +'0', '\0'};
    printUART("Global Count: ");
    printUART(gc);
    //------------------------------------------------------------------
    
    newLine();
      
    
  }
  
}

// print to UART
void printUART(unsigned char * s)
{
  int size = 0;
  while (*(s + size))
  {
    size++;
  }
 
  UARTSend(s, size + 1);
}

void newLine(void)
{
  UARTSend((unsigned char *)"\r\n", 3);
}