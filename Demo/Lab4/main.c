

#include "header.h"

#include "semphr.h"




/* Demo app includes. */
#include "bitmap.h"


//  set this value to non 0 to include the web server

#define mainINCLUDE_WEB_SERVER		1





/*-----------------------------------------------------------*/

/* 
  The time between cycles of the 'check' functionality (defined within the
  tick hook. 
*/
#define mainCHECK_DELAY	( ( portTickType ) 5000 / portTICK_RATE_MS )

// Size of the stack allocated to the uIP task.
#define mainBASIC_WEB_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 3 )

// The OLED task uses the sprintf function so requires a little more stack too.
#define mainOLED_TASK_STACK_SIZE	    ( configMINIMAL_STACK_SIZE + 50 )

//  Task priorities.
#define mainQUEUE_POLL_PRIORITY		    ( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		    ( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		    ( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		    ( tskIDLE_PRIORITY + 2 )
#define mainCREATOR_TASK_PRIORITY           ( tskIDLE_PRIORITY + 3 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )
#define mainGEN_QUEUE_TASK_PRIORITY	    ( tskIDLE_PRIORITY )


//  The maximum number of messages that can be waiting for display at any one time.
  #define mainOLED_QUEUE_SIZE					( 3 )

// Dimensions the buffer into which the jitter time is written. 
  #define mainMAX_MSG_LEN						25

/* 
  The period of the system clock in nano seconds.  This is used to calculate
  the jitter time in nano seconds. 
*/

#define mainNS_PER_CLOCK ( ( unsigned portLONG ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )


// Constants used when writing strings to the display.

#define mainCHARACTER_HEIGHT		    ( 9 )
#define mainMAX_ROWS_128		    ( mainCHARACTER_HEIGHT * 14 )
#define mainMAX_ROWS_96			    ( mainCHARACTER_HEIGHT * 10 )
#define mainMAX_ROWS_64			    ( mainCHARACTER_HEIGHT * 7 )
#define mainFULL_SCALE			    ( 15 )
#define ulSSI_FREQUENCY			    ( 3500000UL )

/*-----------------------------------------------------------*/

/*
 * The task that handles the uIP stack.  All TCP/IP processing is performed in
 * this task.
 */
extern void vuIP_Task( void *pvParameters );

/*
 * The display is written two by more than one task so is controlled by a
 * 'gatekeeper' task.  This is the only task that is actually permitted to
 * access the display directly.  Other tasks wanting to display a message send
 * the message to the gatekeeper.
 */


/*
 * Configure the hardware .
 */
static void prvSetupHardware( void );

/*
 * Configures the high frequency timers - those used to measure the timing
 * jitter while the real time kernel is executing.
 */
extern void vSetupHighFrequencyTimer( void );

/*
 * Hook functions that can get called by the kernel.
 */
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName );
void vApplicationTickHook( void );


/* 
  The queue used to send messages to the OLED task. 
*/
xQueueHandle xOLEDQueue;
xQueueHandle xTrainQueue;
xSemaphoreHandle xSemaphore;
xTaskHandle xTrainComHandle;
xTaskHandle xCurrentTrainHandle;
xTaskHandle xSwitchControlHandle;
xTaskHandle xSerialComHandle;
scheduleD myScheduleD;
trainComD myTrainComD;
serialComD mySerialComD;
currentTrainD myCurrentTrainD;
//soundD mySoundD;
switchControlD mySwitchControlD;


unsigned int globalCount = 0;

// TRAIN INFORMATION
trainInfo trainQueue[5];
unsigned int trainCount = 0;
unsigned int trainSize = 0;
unsigned int freqCount = 0;
unsigned int breakTemperature = 0;
unsigned int fromState = 0;
int passengerCount = 0;

// TRAIN QUEUE
bool toE0 = FALSE;
bool toE1 = FALSE;
bool toE2 = FALSE;
bool toE3 = FALSE;
bool toE4 = FALSE;
bool toW0 = FALSE;
bool toW1 = FALSE;
bool toW2 = FALSE;
bool toW3 = FALSE;
bool toW4 = FALSE;
bool toS0 = FALSE;
bool toS1 = FALSE;
bool toS2 = FALSE;
bool toS3 = FALSE;
bool toS4 = FALSE;
bool toN0 = FALSE;
bool toN1 = FALSE;
bool toN2 = FALSE;
bool toN3 = FALSE;
bool toN4 = FALSE;
bool fromE0 = FALSE;
bool fromE1 = FALSE;
bool fromE2 = FALSE;
bool fromE3 = FALSE;
bool fromE4 = FALSE;
bool fromW0 = FALSE;
bool fromW1 = FALSE;
bool fromW2 = FALSE;
bool fromW3 = FALSE;
bool fromW4 = FALSE;
bool fromS0 = FALSE;
bool fromS1 = FALSE;
bool fromS2 = FALSE;
bool fromS3 = FALSE;
bool fromS4 = FALSE;
bool fromN0 = FALSE;
bool fromN1 = FALSE;
bool fromN2 = FALSE;
bool fromN3 = FALSE;
bool fromN4 = FALSE;
int passengerCount0 = 0;
int passengerCount1 = 0;
int passengerCount2 = 0;
int passengerCount3 = 0;
int passengerCount4 = 0;
unsigned int trainSize0 = 0;
unsigned int trainSize1 = 0;
unsigned int trainSize2 = 0;
unsigned int trainSize3 = 0;
unsigned int trainSize4 = 0;
unsigned int intializedTime0 = 0;
unsigned int intializedTime1 = 0;
unsigned int intializedTime2 = 0;
unsigned int intializedTime3 = 0;
unsigned int intializedTime4 = 0;



// CONTROL
unsigned int trainCounter = 0;
unsigned int traversalCounter = 0;
bool gridlock = FALSE;
unsigned int gridlockDelay = 0;

// ISRs
unsigned int State0 = 0;
unsigned int State1 = 0;
unsigned int State2 = 0;
unsigned int State3 = 0;

// ISR function prototype
void IntGPIOe(void);
void IntGPIOf(void);
void IntUART0(void);
void IntTimer0(void);

int main( void )
{
  //*************************     SETUP     ********************************//  
  prvSetupHardware(); 
 
  vSemaphoreCreateBinary(xSemaphore);  
  /*  
      Create the queue used by the OLED task.  Messages for display on the OLED
      are received via this queue. 
  */
//  //*************************** OLED QUEUE *******************************//
//  xOLEDQueue = xQueueCreate( mainOLED_QUEUE_SIZE, sizeof( xOLEDMessage ) );
      
  //*************************** SEMAPHORE *******************************//

  //**************************** WEBSERVER *******************************//
  /* 
      Exclude some tasks if using the kickstart version to ensure we stay within
      the 32K code size limit. 
  */
  
  #if mainINCLUDE_WEB_SERVER != 0
  {
    /* 
        Create the uIP task if running on a processor that includes a MAC and PHY. 
    */
    
    if( SysCtlPeripheralPresent( SYSCTL_PERIPH_ETH ) )
    {
        xTaskCreate( vuIP_Task, ( signed portCHAR * ) "uIP", mainBASIC_WEB_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
    }
  }
  #endif
    
  //****************     INITIALIZING GLOBALS      *******************//
  myScheduleD.trainCount = &trainCount;  
  myScheduleD.globalCount = &globalCount;
  myScheduleD.trainCounter = &trainCounter;
  myScheduleD.traversalCounter = &traversalCounter;
  myScheduleD.passengerCount = &passengerCount; 
  myScheduleD.freqCount = &freqCount;
  
  myTrainComD.trainCount = &trainCount;
  myTrainComD.breakTemperature = &breakTemperature;
  myTrainComD.fromState = &fromState;
  myTrainComD.freqCount = &freqCount;
  myTrainComD.passengerCount = &passengerCount;
  myTrainComD.globalCount = &globalCount;
  
  mySwitchControlD.trainCount = &trainCount;
  mySwitchControlD.trainCounter = &trainCounter;
  mySwitchControlD.traversalCounter = &traversalCounter;
  mySwitchControlD.gridlock = &gridlock;
  mySwitchControlD.gridlockDelay = &gridlockDelay;
  
  myCurrentTrainD.gridlock = &gridlock;
  myCurrentTrainD.trainCount = &trainCount;
  myCurrentTrainD.breakTemperature = &breakTemperature;
  myCurrentTrainD.trainCounter = &trainCounter;
  myCurrentTrainD.traversalCounter = &traversalCounter;
  myCurrentTrainD.globalCount = &globalCount;
  
 
  mySerialComD.trainCount = &trainCount;
  mySerialComD.breakTemperature = &breakTemperature;
  mySerialComD.gridlock = &gridlock;
  mySerialComD.globalCount = &globalCount;
  mySerialComD.traversalCounter = &traversalCounter;
  mySerialComD.trainCounter = &trainCounter;
  trainQueue[0].toE = &toE0;
  trainQueue[1].toE = &toE1;   
  trainQueue[2].toE = &toE2; 
  trainQueue[3].toE = &toE3; 
  trainQueue[4].toE = &toE4; 
  
  trainQueue[0].toW = &toW0;
  trainQueue[1].toW = &toW1;   
  trainQueue[2].toW = &toW2; 
  trainQueue[3].toW = &toW3; 
  trainQueue[4].toW = &toW4; 
  
  trainQueue[0].toS = &toS0;
  trainQueue[1].toS = &toS1;   
  trainQueue[2].toS = &toS2; 
  trainQueue[3].toS = &toS3; 
  trainQueue[4].toS = &toS4; 
 
  trainQueue[0].toN = &toN0;
  trainQueue[1].toN = &toN1;   
  trainQueue[2].toN = &toN2; 
  trainQueue[3].toN = &toN3; 
  trainQueue[4].toN = &toN4;

  trainQueue[0].fromE = &fromE0;
  trainQueue[1].fromE = &fromE1;   
  trainQueue[2].fromE = &fromE2; 
  trainQueue[3].fromE = &fromE3; 
  trainQueue[4].fromE = &fromE4; 
  
  trainQueue[0].fromW = &fromW0;
  trainQueue[1].fromW = &fromW1;   
  trainQueue[2].fromW = &fromW2; 
  trainQueue[3].fromW = &fromW3; 
  trainQueue[4].fromW = &fromW4; 
  
  trainQueue[0].fromS = &fromS0;
  trainQueue[1].fromS = &fromS1;   
  trainQueue[2].fromS = &fromS2; 
  trainQueue[3].fromS = &fromS3; 
  trainQueue[4].fromS = &fromS4; 
  
  trainQueue[0].fromN = &fromN0;
  trainQueue[1].fromN = &fromN1;   
  trainQueue[2].fromN = &fromN2; 
  trainQueue[3].fromN = &fromN3; 
  trainQueue[4].fromN = &fromN4; 
  
  trainQueue[0].passengerCount = &passengerCount0;
  trainQueue[1].passengerCount = &passengerCount1;
  trainQueue[2].passengerCount = &passengerCount2;
  trainQueue[3].passengerCount = &passengerCount3;
  trainQueue[4].passengerCount = &passengerCount4;
  
  trainQueue[0].trainSize = &trainSize0;
  trainQueue[1].trainSize = &trainSize1;
  trainQueue[2].trainSize = &trainSize2;
  trainQueue[3].trainSize = &trainSize3;
  trainQueue[4].trainSize = &trainSize4;
  
  trainQueue[0].initializedTime = &intializedTime0;
  trainQueue[1].initializedTime = &intializedTime1;
  trainQueue[2].initializedTime = &intializedTime2;
  trainQueue[3].initializedTime = &intializedTime3;
  trainQueue[4].initializedTime = &intializedTime4;
  
  //****************************** TASKS ****************************//  
//  xTaskCreate( vOLEDTask, ( signed portCHAR * ) "OLED", mainOLED_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
  xTaskCreate( vScheduleTask, ( signed portCHAR * ) "Schedule", 1024, (void*)&myScheduleD, tskIDLE_PRIORITY, NULL );
  xTaskCreate( vTrainComTask, ( signed portCHAR * ) "TrainCom", 1024, (void*)&myTrainComD, 1, &xTrainComHandle );
  xTaskCreate( vSwitchControlTask, ( signed portCHAR * ) "SwitchControl", 1024, (void*)&mySwitchControlD, 1, &xSwitchControlHandle );
  xTaskCreate( vSerialComTask, ( signed portCHAR * ) "SerialCom", 1024, (void*)&mySerialComD, tskIDLE_PRIORITY, &xSerialComHandle );
  xTaskCreate( vCurrentTrainTask, ( signed portCHAR * ) "CurrentTrain", 1024, (void*)&myCurrentTrainD, 1, &xCurrentTrainHandle );
//  xTaskCreate( vSoundTask, ( signed portCHAR * ) "Sound", 1024, (void*)&mySoundD, 3, NULL );
  
  /* 
    Configure the high frequency interrupt used to measure the interrupt
    jitter time. 
  */
  vSetupHighFrequencyTimer();

  /* 
    Start the scheduler. 
  */
  vTaskStartScheduler();

  /* Will only get here if there was insufficient memory to create the idle task. */
  return 0;
}


/*
  the OLED Task
*/

//void vOLEDTask( void *pvParameters )
//{
//    xOLEDMessage xMessage;
//    unsigned portLONG ulY, ulMaxY;
//    static portCHAR cMessage[ mainMAX_MSG_LEN ];
//    extern volatile unsigned portLONG ulMaxJitter;
//    unsigned portBASE_TYPE uxUnusedStackOnEntry;
//    const unsigned portCHAR *pucImage;
//
//// Functions to access the OLED. 
//
//    void ( *vOLEDInit )( unsigned portLONG ) = NULL;
//    void ( *vOLEDStringDraw )( const portCHAR *, unsigned portLONG, unsigned portLONG, unsigned portCHAR ) = NULL;
//    void ( *vOLEDImageDraw )( const unsigned portCHAR *, unsigned portLONG, unsigned portLONG, unsigned portLONG, unsigned portLONG ) = NULL;
//    void ( *vOLEDClear )( void ) = NULL;
//  
//  
//    vOLEDInit = RIT128x96x4Init;
//    vOLEDStringDraw = RIT128x96x4StringDraw;
//    vOLEDImageDraw = RIT128x96x4ImageDraw;
//    vOLEDClear = RIT128x96x4Clear;
//    ulMaxY = mainMAX_ROWS_96;
//    pucImage = pucBasicBitmap;
//              
//// Just for demo purposes.
//    uxUnusedStackOnEntry = uxTaskGetStackHighWaterMark( NULL );
//  
//    ulY = ulMaxY;
//    
//    /* Initialise the OLED  */
//    vOLEDInit( ulSSI_FREQUENCY );	
//    
//    while( 1 )
//    {
//      // Wait for a message to arrive that requires displaying.
//      
//      xQueueReceive( xOLEDQueue, &xMessage, portMAX_DELAY );
//  
//      // Write the message on the next available row. 
//      
//      ulY += mainCHARACTER_HEIGHT;
//      if( ulY >= ulMaxY )
//      {
//          ulY = mainCHARACTER_HEIGHT;
//          vOLEDClear();
//      }
//  
//      // Display the message  
//                      
//      sprintf( cMessage, "%s", xMessage.pcMessage);
//      
//      vOLEDStringDraw( cMessage, 0, ulY, mainFULL_SCALE );
// 
//  }
//}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
    ( void ) pxTask;
    ( void ) pcTaskName;
  
    while( 1 );
}

/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
    srand(time(NULL));
    
    /* 
      If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
      a workaround to allow the PLL to operate reliably. 
    */
  
    if( DEVICE_IS_REVA2 )
    {
        SysCtlLDOSet( SYSCTL_LDO_2_75V );
    }
	
    // Set the clocking to run from the PLL at 50 MHz
    
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );
    
    /* 	
      Enable Port F for Ethernet LEDs
            LED0        Bit 3   Output
            LED1        Bit 2   Output 
    */
    
//    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOF );
//    GPIODirModeSet( GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3), GPIO_DIR_MODE_HW );
//    GPIOPadConfigSet( GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3 ), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );	
	//---------------GPIO PORT E PUSH BUTTONS------------------------------//
    //Enable GPIO port E, set pin 0 as an input
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);    
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    
    //Activate the pull-up on GPIO port E
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //Configure GPIO port E as triggering on falling edges
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_FALLING_EDGE);
	
    //Enable interrupts for GPIO port E
    GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    IntEnable(INT_GPIOE);
    
    //---------------------Initialize the OLED display-------------------//
    RIT128x96x4Init(1000000);
	
    //-------------------------SPEAKER-----------------------------------//
    unsigned long ulPeriod; 
	
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // Enable the peripherals used by this example.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    // Set GPIO F0 and G1 as PWM pins.  They are used to output the PWM0 and
    // PWM1 signals.
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
    
    // Compute the PWM period based on the system clock.
    ulPeriod = SysCtlClockGet() / 200;

    // Set the PWM period to 200 (A) Hz.
    PWMGenConfigure(PWM_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);

    // Set PWM0 to a duty cycle of 25% and PWM1 to a duty cycle of 75%.
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, ulPeriod / 4);
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ulPeriod * 3 / 4);

    // Enable the PWM0 and PWM1 output signals.
    PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
//	
//	//---------------------------------TIMER---------------------------------------//
//	//Clear the default ISR handler and install IntTimer0 as the handler:
//    TimerIntUnregister(TIMER0_BASE, TIMER_A);
//    TimerIntRegister(TIMER0_BASE, TIMER_A, IntTimer0);
//
//    //Enable Timer 0    
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
//
//    //Configure Timer 0 and set the timebase to 1 second    
//    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
//    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 2);    
//    
//    //Enable interrupts for Timer0 and activate it
//    IntEnable(INT_TIMER0A);
//    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
//    TimerEnable(TIMER0_BASE, TIMER_A);
//    
    //---------------------------------Function Generator----------------------------------//
    //Enable GPIO port F, set pin 0 as an input
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);

    //Activate the pull-up on GPIO port F
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    //Configure GPIO port F as triggering on falling edges
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
    
    //Enable interrupts for GPIO port F
    GPIOPinIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
    IntEnable(INT_GPIOF);   
	
//    //------------------------------------------UART---------------------------------------//
    // Enable the peripherals used by this example.
//    UARTIntUnregister(UART0_BASE);
//    UARTIntRegister(UART0_BASE, UART0);
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    IntMasterEnable();
    
    // Set GPIO A0 and A1 as UART pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    
    // Enable the UART interrupt.
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	
}


/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
    static xOLEDMessage xMessage = { "PASS" };
    static unsigned portLONG ulTicksSinceLastDisplay = 0;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* 
      Called from every tick interrupt.  Have enough ticks passed to make it
      time to perform our health status check again? 
    */
    
    ulTicksSinceLastDisplay++;
    if( ulTicksSinceLastDisplay >= mainCHECK_DELAY )
    {
       ulTicksSinceLastDisplay = 0;
            
    }
}

////Interrupt Service Routine for GPIO_PORTE_BASE
void IntGPIOe(void) 
{
  //Clear the interrupt to avoid continuously looping here
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);
  
  //Set the Event State for GPIO pin 0
  State0=GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0); // UP 1, north
  State1=GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1); // DOWN 2, south
  State2=GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2); // LEFT 4, west
  State3=GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3); // RIGHT 8, east
  
  //Switches are normally-high, so flip the polarity of the results:
  State0=State0^GPIO_PIN_0;
  State1=State1^GPIO_PIN_1;
  State2=State2^GPIO_PIN_2;
  State3=State3^GPIO_PIN_3;
  
  // fromState 1 = north, 2 = south, 4 = west, 8 = east
  fromState = State0 + State1 + State2 + State3;
  xTaskResumeFromISR(xTrainComHandle);
}

//ISR for GPIO_PORTF_BASE
void IntGPIOf(void) {
  //Clear the interrupt to avoid continuously looping here
  GPIOPinIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
  
  freqCount = freqCount + 1;
}

//// Hyperterm display
void IntUART0(void)
{
    unsigned long ulStatus;

    //
    // Get the interrrupt status.
    //
    ulStatus = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ulStatus);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
    }
}