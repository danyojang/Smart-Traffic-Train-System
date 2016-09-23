#include "header.h"

void IntGPIOe(void);
void Startup(void)
{
	// Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);
				   
	//---------------GPIO PORT E PUSH BUTTONS------------------------------//
    //Clear the default ISR handler and install IntGPIOe as the handler:
    GPIOPortIntUnregister(GPIO_PORTE_BASE);
    GPIOPortIntRegister(GPIO_PORTE_BASE,IntGPIOe);

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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    // Set GPIO F0 and G1 as PWM pins.  They are used to output the PWM0 and
    // PWM1 signals.
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
    
    // Compute the PWM period based on the system clock.
    ulPeriod = SysCtlClockGet() / 200;

    // Set the PWM period to 200 (A) Hz.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ulPeriod);

    // Set PWM0 to a duty cycle of 25% and PWM1 to a duty cycle of 75%.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ulPeriod / 4);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ulPeriod * 3 / 4);

    // Enable the PWM0 and PWM1 output signals.
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
	
	//---------------------------------TIMER---------------------------------------//
	//Clear the default ISR handler and install IntTimer0 as the handler:
    TimerIntUnregister(TIMER0_BASE, TIMER_A);
    TimerIntRegister(TIMER0_BASE, TIMER_A, IntTimer0);

    //Enable Timer 0    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    //Configure Timer 0 and set the timebase to 1 second    
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 2);    
    
    //Enable interrupts for Timer0 and activate it
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
    
    //---------------------------------Function Generator----------------------------------//
    //Clear the default ISR handler and install IntGPIOe as the handler:
    GPIOPortIntUnregister(GPIO_PORTF_BASE);
    GPIOPortIntRegister(GPIO_PORTF_BASE, IntGPIOf);

    //Enable GPIO port E, set pin 0 as an input
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);    
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);

    //Activate the pull-up on GPIO port E
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    //Configure GPIO port E as triggering on falling edges
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
    
    //Enable interrupts for GPIO port E
    GPIOPinIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0);
    IntEnable(INT_GPIOF);   
	
    //------------------------------------------UART---------------------------------------//
    // Enable the peripherals used by this example.
    UARTIntUnregister(UART0_BASE);
    UARTIntRegister(UART0_BASE, UART0);
    
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