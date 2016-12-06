/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware library includes. */
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "sysctl.h"
#include "gpio.h"
#include "grlib.h"
#include "rit128x96x4.h"
#include "osram128x64x4.h"
#include "formike128x128x16.h"
#include "pwm.h"
#include "hw_ints.h"
#include "interrupt.h"

#include "hw_uart.h" // UART
#include "uart.h" // UART




#include "lcd_message.h"


enum myBool { FALSE = 0, TRUE = 1 };
typedef enum myBool bool;

typedef struct {
  // TRAIN INFORMATION
  unsigned int* trainCount;
  
  // CONTROL
  unsigned int* globalCount;
  unsigned int* trainCounter;
  unsigned int* traversalCounter;
  
  // FOR PASSENGERCOUNT
  int* passengerCount;
  unsigned int* freqCount;
 
} scheduleD;

typedef struct {
  bool* toE;
  bool* toW;
  bool* toS;
  bool* toN;
  bool* fromE;
  bool* fromW;
  bool* fromS;
  bool* fromN;
  int* passengerCount;
  unsigned int* trainSize;
  unsigned int* initializedTime;
} trainInfo;

typedef struct {
  // TRAIN INFORMATION
  unsigned int* trainCount;
  unsigned int* breakTemperature;
  unsigned int* fromState;
  unsigned int* freqCount;
  int* passengerCount;
  
  // CONTROL
  unsigned int* globalCount;
} trainComD; 

typedef struct {
  // TRAIN INFORMATION
  unsigned int* trainCount;
  
  // CONTROL
  unsigned int* trainCounter;
  unsigned int* traversalCounter;
  bool* gridlock;
  unsigned int* gridlockDelay;
  
} switchControlD;

typedef struct {
  // TRAIN INFORMATION
  
  unsigned int* trainCount;
  unsigned int* breakTemperature;
  
  
  // CONTROL
  bool* gridlock;
  unsigned int* trainCounter;
  unsigned int* traversalCounter;
  unsigned int* globalCount;
} currentTrainD;
  
//typedef struct {
//  
//} soundD;

typedef struct {
  // TRAIN INFORMATION
  unsigned int* trainCount;
  unsigned int* breakTemperature;
  unsigned int* traversalCounter;
  unsigned int* trainCounter;
  // GRIDLOCK
  bool* gridlock;
  
  // GLOBAL COUNT
  unsigned int* globalCount;
} serialComD;



//static void vOLEDTask( void *pvParameters );
void vScheduleTask( void *pvParameters );
void vTrainComTask( void *pvParameters );
void vSwitchControlTask( void *pvParameters );
void vSerialComTask( void *pvParameters );
void vCurrentTrainTask( void *pvParameters );

// delay function prototype used in Schedule for making more random
void delay(void);

// printing function prototypes
void printTrain(trainInfo train, int horizontalOffset);
void clearDisplay(int horizontalOffset);
void printTrainTwoStatus(unsigned int traversalCounter, unsigned int trainCount);
void printTrainOneStatus(unsigned int globalCount, unsigned int initializedTime);

// UART prototypes
void newLine(void);
void printUART(unsigned char *);

// random function prototype
int rng(int low, int high);
