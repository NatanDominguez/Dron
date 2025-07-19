
//*****************************************************************************
//
//                    NATAN DOMÍNGUEZ & BIEL HORNAS
//
//                    CONTROLADOR BLDC CLOSED LOOP
//                 LABORATORI DE SISTEMES ELECTRÒNICS 1
//                      UNIVERSITAT DE BARCELONA
//
//                        Version:  28/01/2025
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <driverlib/debug.h>
#include <driverlib/fpu.h>
#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>
#include <driverlib/rom.h>
#include <driverlib/sysctl.h>
#include <driverlib/pwm.h>
#include <driverlib/interrupt.h>
#include <inc/hw_ints.h>
#include <driverlib/timer.h>
#include <driverlib/sysctl.h>
#include <inc/hw_ints.h>
#include <driverlib/adc.h>


uint8_t state_l;
uint8_t phase;
uint8_t timerA_flag=0;
uint32_t ticks = 200000;//4ms
uint32_t ticks_limit = 80000;//1,2ms
uint32_t pwm_freq = 2000; //25kHz
uint32_t pulse_width = 450;
uint32_t pulse_with_limit = 280;
uint32_t pendent_rampa = 706;
uint8_t state;
uint8_t restart = 0;
uint8_t fin_rampa = 0;
uint32_t ADC_measure;
float ADC_measure_norm;
uint32_t pulse_width_max = 400;
uint32_t pulse_width_min = 200;
uint32_t pulse_width_pot = 0;

void open_loop(void);
void closed_loop(void);
void PZdetectB(void);
void PZdetectE(void);
void Configure_GPIO(void);
void IntPortFHandler(void);
void Timer0IntHandler(void);
void Timer1IntHandler(void);
void Configure_Timer0();
void Configure_Timer1(uint32_t ticks);
void Configure_PWM(void);
void Configure_ADC(void);
uint32_t read_ADC(void);
void config_PWM_duty(uint32_t pulse_width_pot);

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*************************************


//          FUNCIÓ PRINCIPAL
//_____________________________________________________

int main(void){

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    Configure_GPIO();
    Configure_PWM();
    Configure_Timer0();
    Configure_ADC();

    IntMasterEnable();

    /* INICIAL STATE -PHASE A- (PWMA ON; PWMB OFF; PWMC OFF; AL OFF; BL ON; CL OFF) */
    phase = 1;
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0);           // AL INICIAL STATE OFF
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);  // BL INICIAL STATE ON
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);  // CL INICIAL STATE OFF

    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);//A
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);//B
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT,false);//C


    while(1){

        /*

    EN AQUEST BUCLE CAL FER EL CONTROL DEL MOTOR EN FUNCIÓ DE LA FASE ELÈCTRICA.

    EN UN INICI ELS SALTS ENTRE FASES ELÈCTRIQUES LES CONTROLARÀ EL TIMER A.

    AMB AQUEST TIMER ES FA UNA RAMPA PER ACONSEGUIR ARRENCAR EL MOTOR.

    QUAN FINALITZI LA RAMPA S'INICIARÀ EL CONTROL PER CLOSED-LOOP.

    CALDRÀ CANCEL·LAR ELS CANVIS ENTRE FASES ELÈCTRIQUES A PARTIR DE LES INTERRUPCIONS DEL TIMER I
    PASSAR A CONTROLAR A PARTIR DELS GPIOS DE LA SORTIDA DELS COMPARADORS DEL BEMF.

    A CADA FASE ELÈCTRICA ES PODRÀ MESURAR UN PAS PER ZERO, CAL CONÈIXER QUIN ES DONARÀ
    EN L'ESTAT EN QUÈ ENS TROBEM I PREPARAR EL SISTEMA PER A LA DETECCIÓ AMB EL FLANC CORRESPONENT.


    PER MÉS INFORMACIÓ VEURE COMENTARIS DE LES FUNCIONS RESPECTIVES PELS MODES DE FUNCIONAMENT

        */


        if (fin_rampa){     // CAS CONTROL CLOSED LOOP
        }
        else{               //CAS DEL CONTROL EN LA RAMPA
        }
    }
}


//                  CONTROL FUNCTIONS
//______________________________________________________


void open_loop(void){
    /*
        FUNCIÓ PEL CONTROL OPEN LOOP

        EN FUNCIÓ DE LA FASE ELÈCTRICA ES MODIFIQUEN LES ENTRADES DELS PONTS H

        EN CADA SALT ENTRE FASES HI HA NOMÉS DOS CANVIS I S'ALTERNEN ENTRE PART ALTA I BAIXA DELS PONTS

            PHASE 1: PWMA -> ON; PWMC -> OFF
            PHASE 2: LC -> ON; LB -> OFF
            PHASE 3: PWMB -> ON; PWMA -> OFF
            PHASE 4: LC -> OFF; LA -> ON
            PHASE 5: PWMC -> ON; PWMB -> OFF
            PHASE 6: LA -> OFF; LB -> ON

        _______________________________________
            HA: --------________________
            LA: ____________--------____

            HB: ________--------________
            LB: ----________________----

            HC: ________________--------
            LC: ____--------____________
        _______________________________________


    */

    if(phase > 6)   phase = 1; // ENSURE PERIODICITY

    //LÒGICA PER L'EXCITACIÓ DEL MOTOR

    if(phase == 1){         // PWMA -> ON; PWMC -> OFF
        PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);//A
        PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT,false);//C
    }
    else if(phase == 2){    // BL -> OFF; CL -> ON
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
    }
    else if(phase == 3){    // PWMA -> OFF; PWMB -> ON
        PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);//A
        PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);//B
    }
    else if(phase == 4){    // AL -> ON; CL -> OFF
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);
    }
    else if(phase == 5){    // PWMB -> OFF; PWMC -> ON
        PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);//B
        PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT,true);//C
    }
    else{   //phase = 6 ||     AL -> OFF; BL -> ON
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    }
}


void closed_loop(void){

/*

        FUNCIÓ PEL CONTROL CLOSED LOOP

        S'APLICA LA MATEIXA LÒGICA QUE EL CONTROL OPEN LOOP, AMB LA DIFERÈNCIA
        QUE EL SALT ENTRE FASES VE DONAT PER LA DETECCIÓ DEL PAS PER ZERO DEL BEMF

        QUAN UNA DE LES FASES ESTA FLOTANT TÉ LLOC EL PAS PER ZERO, CAL
        ACTIVAR LA INTERRUPCIÓ RESPECTIVA PER A DETECTAR-LA

            PHASE 1: DETECT C FALLING - (PWMA -> ON; PWMC -> OFF)
            PHASE 2: DETECT B RISING - (LC -> ON; LB -> OFF)
            PHASE 3: DETECT A FALLING - (PWMB -> ON; PWMA -> OFF)
            PHASE 4: DETECT C RISING - (LC -> OFF; LA -> ON)
            PHASE 5: DETECT B FALLING - (PWMC -> ON; PWMB -> OFF)
            PHASE 6: DETECT A RISING - (LA -> OFF; LB -> ON)

        _______________________________________
            HA: --------_|____________|_
            LA: __________|_--------_|__

            HB: ______|_--------_|______
            LB: ----_|____________|_----

            HC: _|____________|_--------
            LC: __|_--------_|__________
        _______________________________________

*/

    ADC_measure = read_ADC();

    config_PWM_duty(ADC_measure);

    // INITIAL RESET

    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);    // HA RESET
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, false);    // HB RESET
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT,false);     // HC RESET
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0);       // LA RESET
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);       // LB RESET
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0);       // LC RESET

    if(phase > 6)   phase = 1; // ENSURE PERIODICITY


    if(phase == 1){         // READ PHASE C FALLING
        // PE5 -> FALLING
        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_5);

        PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);                 // HA
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);          // LB
    }

    else if(phase == 2){    // READ PHASE B RISING
        // PE4 -> RISING

        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_4);

        PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);                 // HA
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);          // LC
    }

    else if(phase == 3){    // READ PHASE A FALLING
        // PB1 -> FALLING
        GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
        GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_1);

        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);          // LC
        PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);                 // HB
    }

    else if(phase == 4){    // READ PHASE C RISING
        // PE5 -> RISING
        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_RISING_EDGE);
        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_5);

        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);          // LA
        PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);                 // HB

    }

    else if(phase == 5){    // READ PHASE B FALLING
        // PE4 -> FALLING
        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_4);

        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);          // LA
        PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT,true);                  // HC

    }
    else{                   // READ PHASE A RISING
        // PB1 -> RISING
        GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_RISING_EDGE);
        GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_1);

        PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT,true);                  // HC
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);          // LB

    }
}


uint32_t read_ADC(void){
    uint32_t measure;

    ADCProcessorTrigger(ADC0_BASE, 1);  // GENEREM FLAG PER A FER UNA MESURA

    while(!ADCIntStatus(ADC0_BASE, 1, false)){} // ESPEREM A QUE ES FACI LA MESURA

    ADCSequenceDataGet(ADC0_BASE, 1, &measure); // GUARDEM  LA MESURA A LA DIRECCIÓ DE "MEASURE"

    ADCIntClear(ADC0_BASE, 1);  // NETEJEM FLAGS

    return measure;

}


void config_PWM_duty(uint32_t pulse_width_pot){

    pulse_width_pot=pulse_width_pot/2;

    if (pulse_width_pot<=100){
        pulse_width_pot=100;
    }
    else if (pulse_width_pot>=640){
        pulse_width_pot=640;
    }
//pulse_width_act = pulse_with_limit + (pos_pot - 0.5)*(pulse_width_max-pulse_width_min);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, pulse_width_pot);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, pulse_width_pot);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pulse_width_pot);
}

//                  CONFIG FUNCTIONS
//______________________________________________________


void Configure_GPIO(void){

    // Enable the GPIO port F / C / A

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)){}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)){}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)){}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){}

    //
    //INICIALIZE BUTTON PF2
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    //Habilita pull-up
    GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    IntEnable(INT_GPIOF);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTF_BASE,IntPortFHandler); // Registrar la ISR para el puerto F
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_4);  // Habilitar interrupci�n para PF4


    // OUTPUT INIT
        /*
         * LA -> PC4 (OUTPUT)
         * LB -> PF2 (OUTPUT)
         * LC -> PA6 (OUTPUT)
         */

    // PC5
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_PIN_4);  //INICIAL STATE OFF

    // PF2
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);  //INICIAL STATE OFF

    // PA6
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);  //INICIAL STATE OFF


    // INPUT PAS ZERO

        /*
         * PB1 -> PZA (INPUT)
         * PE4 -> PZB (INPUT)
         * PE5 -> PZC (INPUT)
        */

    // PB1

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_1 ,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_RISING_EDGE);
    GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_1);
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_1);

    // PE4

    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_INT_PIN_4);
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_4);

    // PE5

    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTE_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_RISING_EDGE);
    GPIOIntDisable(GPIO_PORTE_BASE, GPIO_INT_PIN_5);
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_5);


    GPIOIntRegister(GPIO_PORTB_BASE, PZdetectB);
    GPIOIntRegister(GPIO_PORTE_BASE, PZdetectE);
    IntEnable(INT_GPIOB);
    IntEnable(INT_GPIOE);
}


void Configure_PWM(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {

    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1))
    {

    }


    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinConfigure(GPIO_PA7_M1PWM3);

    GPIOPinTypePWM(GPIO_PORTC_BASE,GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_3);
    GPIOPinTypePWM(GPIO_PORTA_BASE,GPIO_PIN_7);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);


    /*
    FASE A ------------- PC5 -> GEN 3   PWM0
    FASE B ------------- PF3 -> GEN 3   PWM1
    FASE C ------------- PA7 -> GEN 1   PWM1
    */

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, pwm_freq);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, pwm_freq);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, pwm_freq);
    //
    // Set the pulse width of PWM0 for a 25% duty cycle.
    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, pulse_width);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, pulse_width);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pulse_width);

    //
    // Start the timers in corresponding generators.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    //
    // Enable the outputs.
    //

    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);
    PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);


}


void Configure_Timer0(){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    //
    // Wait for the Timer0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //
    // Set the count time for the the one-shot timer (TimerA).
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, ticks);
    //
    //

    TimerIntRegister(TIMER0_BASE,TIMER_A,Timer0IntHandler);
    //
    // Enable the timers.


    // evento de interrupcion que salta cuando el timer cuenta hasta ticks
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER0_BASE, TIMER_A);

    IntEnable(INT_TIMER0A);
    //

}

void Configure_Timer1(uint32_t ticks){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    //
    // Wait for the Timer0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1))
    {
    }

    TimerConfigure(TIMER1_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC);
    //
    // Set the count time for the the one-shot timer (TimerA).
    //
    TimerLoadSet(TIMER1_BASE, TIMER_A, ticks);
    //
    //

    TimerIntRegister(TIMER1_BASE,TIMER_A,Timer1IntHandler);
    //
    // Enable the timers.


    // evento de interrupcion que salta cuando el timer cuenta hasta ticks
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER1_BASE, TIMER_A);

    IntEnable(INT_TIMER1A);
    //

}


void Configure_ADC(void){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);


    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    ADCSequenceDisable(ADC0_BASE, 1);

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH5 | ADC_CTL_IE | ADC_CTL_END);

    ADCIntClear(ADC0_BASE, 1);

}


//               HANDLER FUNCTIONS
//______________________________________________________


void PZdetectB(void){

    // PZA INTERRUPT HANDLER

    IntMasterDisable();

    GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_1);
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_1);

    if(GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_1) & GPIO_PIN_1){  // A RISING
        phase = 1;
    }
    else{                                                       // A FALLING
        phase = 4;
    }

    IntMasterEnable();
    closed_loop();

}


void PZdetectE(void){

    IntMasterDisable();
    uint32_t status = GPIOIntStatus(GPIO_PORTE_BASE,true);

    if ((status & GPIO_PIN_4)==GPIO_PIN_4){
        GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_4);
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);

        if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4) & GPIO_PIN_4){  // B RISING
            phase = 3;
        }
        else{                                                       // B FALLING
            phase = 6;
        }
    }
    else if((status & GPIO_PIN_5)==GPIO_PIN_5){
        GPIOIntDisable(GPIO_PORTE_BASE, GPIO_PIN_5);
        GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_5);

        if(GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_5) & GPIO_PIN_5){  // C RISING
            phase = 5;
        }
        else{                                                       // C FALLING
            phase = 2;
        }

    }

    IntMasterEnable();

    closed_loop();


}


void IntPortFHandler(void){

    IntMasterDisable();
    uint32_t status = GPIOIntStatus(GPIO_PORTF_BASE,true);
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_INT_PIN_4);

    if ((status & GPIO_PIN_4)==GPIO_PIN_4){
        restart ^= 1;
    }
    IntMasterEnable();
}


void Timer0IntHandler(void){
    IntMasterDisable();
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //

    if (phase==1 && pulse_width >= pulse_with_limit){
            pulse_width--;


            ticks -=pendent_rampa;
            TimerLoadSet(TIMER0_BASE, TIMER_A, ticks);

            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, pulse_width);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, pulse_width);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, pulse_width);
    }

    else if(pulse_width <= pulse_with_limit){   //la rampa ha finalitzat
        // passem a closed-loop (desactivem aquest timer)

        // puede que falte hacer phase-- para sincronizar con el closed loop
        fin_rampa = 1;

        TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
        ADCSequenceEnable(ADC0_BASE, 1);

        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_5);
        GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);
        GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_4);
        GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
        GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_1);

    }

    phase++;

    IntMasterEnable();

    open_loop();
}


/*void Timer1IntHandler(void){
    IntMasterDisable();
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //

    IntMasterEnable();
}
*/
//*****************************************************************************
