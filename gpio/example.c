//
// ODROID-C GPIO(LED) / ADC Test Application.
//
// Defined port number is wiringPi port number.
//
// Compile : gcc -o <create excute file name> <source file name> -lwiringPi -lwiringPiDev -lpthread
// Run : sudo ./<created excute file name>
//
//------------------------------------------------------------------------------------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
 
#include <unistd.h>
#include <string.h>
#include <time.h>
 
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringSerial.h>
#include <lcd.h>
 
//------------------------------------------------------------------------------------------------------------
//
// Global handle Define
//
//------------------------------------------------------------------------------------------------------------
 
#define DATA_UPDATE_PERIOD  100 // 100ms
 
//------------------------------------------------------------------------------------------------------------
//
// ADC:
//
//------------------------------------------------------------------------------------------------------------
#define PORT_ADC1   0   // ADC.AIN0
 
static int adcValue = 0;
 
//------------------------------------------------------------------------------------------------------------
//
// LED:
//
//------------------------------------------------------------------------------------------------------------
static int ledPos = 0;
 
const int ledPorts[] = {
    24, // GPIOX.BIT0(#97)
    23, // GPIOX.BIT11(#108)
    22, // GPIOX.BIT3(#100)
    21, // GPIOX.BIT4(#101)
    14, // GPIOX.BIT8(#105)
    13, // GPIOX.BIT9(#106)
    12, // GPIOX.BIT10(#107)
    3,  // GPIOX.BIT18(#115)
    2,  // GPIOX.BIT19(#116)
    0,  // GPIOY.BIT8(#88)
    7,  // GPIOY.BIT3(#83)
 
    1,  // GPIOY.BIT7(#87)
    4,  // GPIOX.BIT7(#104)
    5,  // GPIOX.BIT5(#102)
    6,  // GPIOX.BIT6(#103)
    10, // GPIOX.BIT20(#117)
    11, // GPIOX.BIT21(#118)
    26, // GPIOX.BIT2(#99)
    27, // GPIOX.BIT1(#98)
};
 
#define MAX_LED_CNT sizeof(ledPorts) / sizeof(ledPorts[0])
 
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//
// system init
//
//------------------------------------------------------------------------------------------------------------
int system_init(void)
{
    int i;
 
    // GPIO Init(LED Port ALL Output)
    for(i = 0; i < MAX_LED_CNT; i++)    pinMode (ledPorts[i], OUTPUT);
 
    return  0;
 }
 
//------------------------------------------------------------------------------------------------------------
//
// board data update
//
//------------------------------------------------------------------------------------------------------------
void boardDataUpdate(void)
{
    int i;
 
    // adc value read
    if((adcValue = analogRead (PORT_ADC1)))    {
        ledPos = (adcValue * MAX_LED_CNT * 1000) / 1024;
        ledPos = (MAX_LED_CNT - (ledPos / 1000));
    }
    else
        ledPos = 0;
 
    //  LED Control
    for(i = 0; i < MAX_LED_CNT; i++)    digitalWrite (ledPorts[i], 0); // LED All Clear
    for(i = 0; i < ledPos;      i++)    digitalWrite (ledPorts[i], 1); // LED On
}
 
//------------------------------------------------------------------------------------------------------------
//
// Start Program
//
//------------------------------------------------------------------------------------------------------------
int main (int argc, char *argv[])
{
    static int timer = 0 ;
 
    wiringPiSetup ();
    printf("Initializing...\n"); 
    if (system_init() < 0)
    {
        fprintf (stderr, "%s: System Init failed\n", __func__);     return -1;
    }
    printf("Initialized...\n"); 
    for(;;)    {
 
        if (millis () < timer)  continue ;
 
        timer = millis () + DATA_UPDATE_PERIOD;
 
        // All Data update
        boardDataUpdate();
    }
 
    return 0 ;
}
 
