/*
 * File:   main.c
 * Author: Rayne
 *
 * Created on 08 March 2024, 21:59
 */


#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#pragma config OSC = HS //High-speed Resonator
#pragma config WDT = OFF //Watchdog timer off
#pragma config LVP = OFF //Low voltage programing disabled
#pragma config PWRT = ON //Power up timer on
#define _XTAL_FREQ 10000000 // define clock frequency for __delay_10ms()

#define A1 LATBbits.LATB0  //sets the motor bits to be able to be asigned with the variables for increased readability
#define A2 LATBbits.LATB1
#define B1 LATAbits.LATA4
#define B2 LATAbits.LATA5

#define REnc PORTCbits.RC5 //sets the encoder bits to  allow us to read to them
#define LEnc PORTCbits.RC0

#define LED1 LATBbits.LATB2    //Define LED1
#define LED2 LATBbits.LATB3    //Define LED2
#define LED3 LATBbits.LATB4    //Define LED3
#define LED4 LATBbits.LATB5    //Define LED4

#define LOOKUP [[1 0 0 0 0 0] [1 1 1 1 1 1]]
#define ENCTURNVAL 8 //defines constant for the number of rotations that is required to turn 90 degrees
#define COLLISION_THRESH 800 //defines constant to begin to avoid collision advoidance
void Setup(void);               //Sets up the values needed to operate the robot

void FlashLED(void);            //flashes the LEDs on or off for 5 seconds
void allLED(int val);           //sets every LED to either on or off based on if one or zero entered.

void MotorForwards(void);       //motor functions, does what they say on the tin
void TurnRight(void);
void TurnLeft90(void);
void EncoderChecker(int encVal);
void MotorBrake(void);
unsigned int readLADC(void);    //Read ADC
unsigned int readRADC(void);    //Read ADC

void wait10ms(int del);         //delay function

void main(void) {
    Setup();
    return;
}

void Setup(){
    unsigned char markspace=500;     //mark space value for 8 PWM (50% mark space ratio)
    TRISCbits.RC1=0;       //set CCP1(pin13) to an output pin
    TRISCbits.RC2=0;       //set CCP1(pin13) to an output pin
    TRISB=0b00000000;      //set Motor pins to be output pins
    TRISA=0b11001111;      //set Motor pins to be output pins
    PR2 = 0b11111111;     //set period of PWM  
    T2CON = 0b00000111 ;   //Timer 2(TMR2) on, Prescaler = 16  
   
    CCP1CON = (0x0c);        //0x0c enables PWM module CCP1
    CCP2CON = (0x0c);        //0x0c enables PWM module CCP2

    CCPR1L = markspace;  //Load duty cycle into CCP1CON, PWM begins (Right)
    CCPR2L = markspace;  //Load duty cycle into CCP2CON, PWM begins (Left)
   
    ADCON1=0b00001101;  //Set voltage reference and port A0 as analogue input
    ADCON2=0b10000010; // Fosc/32, A/D result right justified
    LATB=0;             //Turn all Leds off
    FlashLED();
    
    //NEED TO ALTER CODE TO SET UP PARTS CORRECTLy FOR I2C
}

unsigned int readRADC() {

    ADCON0 = 0b00000111; //select A/D channel AN0,start conversion
   
    while (ADCON0bits.GO){}; //do nothing while conversion in progress
    return ((ADRESH << 8) + ADRESL); //Combines high and low A/D bytes into one
}                                 // value and returns this A/D value 0-1023

unsigned int readLADC() {

    ADCON0 = 0b00000011; //select A/D channel AN0,start conversion
    while (ADCON0bits.GO){}; //do nothing while conversion in progress
    return ((ADRESH << 8) + ADRESL); //Combines high and low A/D bytes into one
    }                                 // value and returns this A/D value 0-1023


void FlashLED(){
    for (int i=0;i<5;i++){
        allLED(1);             //turn all LEDs on
        wait10ms(50);          //wait 1 second
        allLED(0);             //turn all LEDs off
        wait10ms(50);          //wait 1 second
}
}
void allLED(int val){ //sets every LED to either on or off based on if one or zero entered.
    LED1=val; //sets LED1 to the value specified
    LED2=val; //sets LED2 to the value specified etc...
    LED3=val;
    LED4=val;
    return;
}

void MotorForwards(){
    CCP1CON=0b00001100; //enables PWMA
    CCP2CON=0b00001100; //enables PWMB

    A1=0;
    A2=1; //sets right motor to forwards
   
    B1=0;
    B2=1;
          //sets left  motor to forwards
    return;
}
void TurnRight(){
    CCP1CON=0b00001100; //enables PWMA
    CCP2CON=0b00001100; //enables PWMB

    A1=1;
    A2=0; //sets right motor to reverse
    B1=0;
    B2=1;
          //sets left  motor to forwards
    return;
}

void TurnRight45(){
    TurnRight();
    EncoderChecker(ENCTURNVAL);
    return;
}
void TurnRight180(){
    TurnRight();
    EncoderChecker(4*ENCTURNVAL);
    return;
}

void EncoderChecker(int encVal){
    unsigned int encCountL=0;
    unsigned int encCountR=0;
    while(encCountL<encVal || encCountR<encVal){ //loops until both constants are met
        encCountR=encCountR+REnc;           //sums encoder values. The value is 1 when a turn is complete, and 0 otherwise. Hence, will only increment it if true
        encCountL=encCountL+LEnc;
        if (encCountL >= encVal){       //Checks left value against the turn constant and brakes left motor if met
            B1=1;
            B2=1;
        }
        if (encCountR >= encVal){      //Checks right value against the turn constant and brakes right motor if met
            A1=1; 
            A2=1;
        }
        wait10ms(1);        
    };

}

void MotorBrake(){
    A1=1;
    A2=1; //sets right motor to brake
   
    B1=1;
    B2=1; //sets left motor to brake
    return;
}
void MotorCoast(){
    CCP1CON=0b00001100; //enables PWMA
    CCP2CON=0b00001100; //enables PWMB
    A1=0;
    A2=0;
    B1=0;
    B2=0; //tells all motors to coast
    return;
}
void wait10ms(int del){     //delay function
    unsigned int c;
    for(c=0;c<del;c++)
        __delay_ms(10);
    return;
}