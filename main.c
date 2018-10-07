/**********************************/
//
//PRACTICA GRUPAL BOXEADOR
//SAUL ALEXANDER LOPEZ HERRERA
//PABLO JAVIER AVILA CULAJAY
//PERCY MATHEW JACOB ORELLANA
//LABORATORIO DE MICROCONTROLADORES
//
//
/**********************************/

/************************LIBRERIAS DE C***************************************/


#include <stdint.h>
#include <stdbool.h>



/************************LIBRERIAS DE TIVAWARE*************************************/
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
//#include "inc/hw_ints.h"
/**
 * main.c
 */

/***********************METODOS A UTILIZAR*****************************************/
void Configuracion(void);
void PWM_Config(void);
void UART_Config(void);

void Servo_Base(int Ang);       //PB5
void Servo_Brazo(int Ang);      //PD0
void Servo_Mano(int Ang);       //PD1

void Maquina_estado(int estado);        //Switch-CASE PARA EL CAMBIO DE ESTADO DE LA MAQUINA
void Estado0(uint8_t instruccion);                     //ESTADO 0, AUTOMATICO
void Estado1(uint8_t instruccion);                     //ESTADO 1, MANUAL
void Estado2(uint8_t instruccion);                     //ESTADO 2, BOXEADOR
/***********************VARIABLES A USAR PWM******************************************/
#define frecuencia 50
uint32_t Load;
uint32_t PWMClk;
volatile uint8_t dato;
/***********************MAS VARIBLES****************************************/
uint8_t Ang1=45;
uint8_t Ang2=45;
uint8_t Ang3=45;
float carga=1;
int pos=0;
double T=0.5;
int  pulso=0;
volatile uint8_t Posicion_Maquina=1;
/*******************************ESTRUCTURA PARA EL CAMBIO DE ESTADO*****************************************************/
struct Maquina{
    int Anterior;
    int Actual;
    int Siguiente;
};
typedef struct Maquina MaquinaEstados;
MaquinaEstados ME[3]={
    {2,0,1},
    {0,1,2},
    {1,2,0}
};
/***************************************************************************************/
void Configuracion(void){
        SysCtlClockSet(SYSCTL_XTAL_16MHZ|SYSCTL_SYSDIV_2_5);                          // Configurar reloj 80Mhz.
        IntMasterEnable();
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                        //Habilitar periferico GPIO PORT F
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                        //Habilitar periferico GPIO PORT B
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);                        //Habilitar periferico GPIO PORT D
        /*RUTINA DESBLOQUEO GPIO F PIN O*/
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
        HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
        /*********************************/
        /*************HABILITACION DE PUERTO F PUSHs PLACA TIVA ********************/
        GPIOPinTypeGPIOInput(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);                //Habilitar PUSH de entradas.
        GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);//Configura el modo de los pines de entrada
        //*****************INTERRUPCION PARA LOS PUSH **************************//
        IntEnable(INT_GPIOF);
        GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_FALLING_EDGE);
        GPIOIntEnable(GPIO_PORTF_BASE,GPIO_INT_PIN_0|GPIO_INT_PIN_4);
        IntPrioritySet(INT_GPIOF,0);
}
/****************************************************************************************/
/************************************CONFIGURACION DE PWM********************************/
void PWM_Config(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);         //Habilitar periferico PWM1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);         //Habilitar periferico PWM0
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);                //CLOCK PWM ES 80Mhz/64
    GPIOPinTypePWM(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1); //ASIGNAR A GPIO LOS PINES PD0 Y PD1 SON PWM
    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_5);            //ASIGNAR A GPIO EL PIN PB5 ES PWM
    GPIOPinConfigure(GPIO_PD0_M1PWM0);  //ASIGNO EL CONTROLADOR PWM AL PIN
    GPIOPinConfigure(GPIO_PD1_M1PWM1);  //ASIGNO EL CONTROLADOR PWM AL PIN
    GPIOPinConfigure(GPIO_PB5_M0PWM3);  //ASIGNO EL CONTROLADOR PWM AL PIN
    PWMClk = SysCtlClockGet()/64;   //PWM CLOCK DEL GEN0
    Load = (PWMClk/frecuencia)-1;   //REPRESENTACION NUMERICA DE LA CARGA DEL PWM
    PWMGenConfigure(PWM1_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN);     //MODO DE CONTEO DEL GENERADOR 0
    PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_DOWN);     //MODO DE CONTEO DEL GENERADOR 1
    PWMGenPeriodSet(PWM1_BASE,PWM_GEN_0,Load);  //LA CARGA DEL PWM
    PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,Load);  //LA CARGA DEL PWM
    PWMOutputState(PWM1_BASE,PWM_OUT_0_BIT,true);   //PWM1_0 - PD0, HABILITAR
    PWMOutputState(PWM1_BASE,PWM_OUT_1_BIT,true);   //PWM1_1 - PD1, HABILITAR
    PWMOutputState(PWM0_BASE,PWM_OUT_3_BIT,true);   //PWM0_3 - PB5, HABILITAR
    PWMGenEnable(PWM1_BASE,PWM_GEN_0);      //HABILITO EL GENERADOR, DEL PWM1_0 Y PWM1_1
    PWMGenEnable(PWM0_BASE,PWM_GEN_1);      //HABILITO EL GENERADOR, DEL PWM0_3
}
/*****************************************************************************************/
/***********************************CONFIGURACION -UART0**********************************/
void UART_Config(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8));


    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE,UART_INT_RX|UART_INT_RT);
}
/*****************************************************************************************/

//********************RUTINAS DE INTERRUPCION********************************/
//*************************************************************************************/
void GPIO_INT_Handler(void){
    pulso++;
    int statusInt;
    //********************RUTINA INTERRUPCION*****************************************//
    statusInt=GPIOIntStatus(GPIO_PORTF_BASE,true);
    GPIOIntClear(GPIO_PORTF_BASE,statusInt);
    //********************************************************************************//
    if(statusInt==1){
        Posicion_Maquina = ME[Posicion_Maquina].Siguiente;
    }
    if(statusInt==16){
        Posicion_Maquina = ME[Posicion_Maquina].Anterior;
    }
    switch (Posicion_Maquina){              //Envio el estado actual de la maquina
        case 0:
            UARTCharPut(UART0_BASE,'0');
            break;
        case 1:
            UARTCharPut(UART0_BASE,'1');
            break;
        case 2:
            UARTCharPut(UART0_BASE,'2');
            break;
    }

}
//************************************************************************************/
void UARTIntHandler(void){
    //********************RUTINA INTERRUPCION*****************************************//
    uint32_t Status;
    Status = UARTIntStatus(UART0_BASE,true);
    UARTIntClear(UART0_BASE,Status);
    //********************************************************************************//

    while(UARTCharsAvail(UART0_BASE)){
        dato = UARTCharGetNonBlocking(UART0_BASE);;
        SysCtlDelay(0.01*(SysCtlClockGet())/3);
    }
    //UARTCharPut(UART0_BASE,dato);
    Maquina_estado(Posicion_Maquina);

}

int main(void)
{
    Configuracion(); //CONFIGURACION PERIFERICOS
    PWM_Config();    //CONFGGURACION PWM
    UART_Config();   //CONFGURACION UART0
    //UARTCharPut(UART0_BASE,ME[Posicion_Maquina].Actual);
    while(true){

    }


}
void Maquina_estado(int estado){
    switch(estado){
    case 0 :
        Estado0(dato);
        break;
    case 1 :
        Estado1(dato);
        break;
    case 2 :
        Estado2(dato);
        break;
    }

}
void Estado0(uint8_t instruccion){              //Estado Automatico
    switch(instruccion){
    case '1':
        ////Movimiento Tirar Caja 1.
        break;
    case '2':
        ////Movimiento Tirar Caja 2.
        break;
    case '3':
        ////Movimineto Tirar Caja 3.
        break;
    }
}
void Estado1(uint8_t instruccion){              //Estado Manual
    switch (instruccion){
        case('W'):                         //BRAZO
            if(Ang1==180){break;}else{
                Ang1++;}
            Servo_Brazo(Ang1);
            break;
        case('S'):
            if(Ang1==0){break;}else{
            Ang1--;}
            Servo_Brazo(Ang1);
            break;
        case('A'):                          //BASE
            if(Ang2==180){break;}else{
                Ang2++;}
            Servo_Base(Ang2);
            break;
        case('D'):
            if(Ang2==0){break;}else{
                Ang2--;}
            Servo_Base(Ang2);
            break;
        case('R'):                          //MANO
            if(Ang3==180){break;}else{
                Ang3++;}
            Servo_Mano(Ang3);
            break;
        case('F'):
            if(Ang3==0){break;}else{
                Ang3--;}
            Servo_Mano(Ang3);
            break;
    }
    //SysCtlDelay(0.01*SysCtlClockGet()/3);
}
void Estado2(uint8_t instruccion){              //Estado Boxeador

}
/***********************METODOS PARA CARGA DE SERVOS, RECIBEN ANGULO*******************/
void Servo_Base(int Ang){
    float trabajo = (0.01028*Ang)+0.6;
    PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,trabajo*Load/20);
}
void Servo_Brazo(int Ang){
    float trabajo = (0.01055*Ang)+0.55;
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_0,trabajo*Load/20);
}
void Servo_Mano(int Ang){
    float trabajo = (0.01111*Ang)+0.5;
    PWMPulseWidthSet(PWM1_BASE,PWM_OUT_1,trabajo*Load/20);
}
/****************************************************************************************/
