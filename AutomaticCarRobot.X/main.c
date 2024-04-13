/*
 * File:   main.c
 * Author: Group 14
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

#define ENCTURNVAL 8 //defines constant for the number of rotations that is required to turn 90 degrees
#define COLLISION_THRESH 800 //defines constant to begin to avoid collision advoidance
#define K 4

void Setup(void);
unsigned int readRADC(void);
unsigned int readLADC(void);
void FlashLED(void);
void allLED(int val);
void MotorForwards(void);
void TurnRight(void);
void TurnRight45(void);
void TurnRight180(void);
void EncoderChecker(int encVal);
void MotorBrake(void);
void MotorCoast(void);
void wait10ms(int del);
void MotorSpeed(void);
void MotorAngle(void);
void AddSpeed(int u, int speedOrAngle);
void SwitchLane(void);
int DetectPLine(void);
void I2C_Initialise(void);
void I2C_checkbus_free(void);
void I2C_Start(void);
void I2C_RepeatedStart(void);
void I2C_Stop(void);
void I2C_Write(unsigned char write);
unsigned char I2C_Read(void);
unsigned char ReadSensorArray();
void AutomaticLineFollow(int a);
void MotorPath(void);


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

    TRISCbits.RC3=1; //Set the RC3 pin as input
    TRISCbits.RC4=4; //Set the RC4 pin as output
    I2C_Initialise(); //Initialise the I2C

    FlashLED(); //Flash the LED after set up

    //Set up complete
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

void FlashLED(){       //flashes the LEDs on and off for 5 seconds
    for (int i=0;i<5;i++){
        allLED(1);             //turn all LEDs on
        wait10ms(50);          //wait 1 second
        allLED(0);             //turn all LEDs off
        wait10ms(50);          //wait 1 second
}
}

void allLED(int val){ //sets every LED to either on or off based on if one or zero entered.
    LED1=val; //sets LED1 to the value specified
    LED2=val; //sets LED2 to the value specified
    LED3=val; //sets LED3 to the value specified
    LED4=val; //sets LED4 to the value specified
}

void MotorForwards(){
    CCP1CON=0b00001100; //enables PWMA
    CCP2CON=0b00001100; //enables PWMB

    A1=0; //sets right motor to reverse
    A2=1; //sets right motor to forwards

    B1=0; //sets left motor to reverse
    B2=1; //sets left  motor to forwards

}

void TurnRight(){
    CCP1CON=0b00001100; //enables PWMA
    CCP2CON=0b00001100; //enables PWMB

    A1=1; //sets right motor to forwards
    A2=0; //sets right motor to reverse
    B1=0; //sets left motor to reverse
    B2=1; //sets left  motor to forwards

    return;
}

void TurnRight45(){
    TurnRight(); //TurnRight() function
    EncoderChecker(ENCTURNVAL); //The encoder turn value is sent to the EncoderChecker
    return;
}

void TurnRight180(){
    TurnRight();
    EncoderChecker(4*ENCTURNVAL); //The encoder turn value is multiplied by gain 4 and is sent to the EncoderChecker
    return;
}

void EncoderChecker(int encVal){
    unsigned int encCountL=0; //Initialize the left encoder count
    unsigned int encCountR=0; //Initialize the right encoder count
    while(encCountL<encVal || encCountR<encVal){  //loops until both constants are met
        encCountR=encCountR+REnc;           //sums encoder values. The value is 1 when a turn is complete, and 0 otherwise. Hence, will only increment it if true
        encCountL=encCountL+LEnc;
        if (encCountL >= encVal){       //Checks if left encoder count has reaches "encVal"
            B1=1; //If the left encoder count meets or exceeds "encVal", brake the left motor
            B2=1;
        }
        if (encCountR >= encVal){      //Checks if the right encoder count has reaches "encVal"
            A1=1;  // If right encoder count meets or exceeds encVal, brake the right motor
            A2=1;
        }
        wait10ms(1); //wait for a short duration before checking again
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
        __delay_ms(10);     //delay for 1s
    return;
}

void MotorSpeed(){
    int distance; //variable to store distance
    int error;    //variable to store error
    int u;        //variable to store control input
    if (readRADC()>=COLLISION_THRESH){  //check if there are obstacles detected
        MotorBrake();  // Brake if obstacle detected
    } else{
        MotorForwards();  //Move forwards if no obstacle detected
        distance=readRADC();  //distance is read from sensor
        wait10ms(1);          //wait for a short time
        distance=distance-readRADC();        //calculates change of distance. if negative, it is too slow and if positive
        error=0-distance/10;                //calculates speed off mm/ms=m/s
        u=K*error;  //K is a constant representing the control gain
        AddSpeed(u,1); //The '1' indicates forward motion
    }
}

void MotorAngle() {
    int angle; //variable to store angle
    int error; //variable to store error
    int u;     //variable to store control input
    unsigned char colourArray = ReadSensorArray(); //Import array data
    switch (colourArray) {   // Angle is determined based on the sensor array data
        case 0b11111110:
            angle = 12;
            break;
        case 0b11111100:
            angle = 10;
            break;
        case 0b11111101:
            angle = 9;
            break;
        case 0b11111001:
            angle = 7;
            break;
        case 0b11111011:
            angle = 5;
            break;
        case 0b11110011:
            angle = 3;
            break;
        case 0b11110111:
            angle = 2;
            break;
        case 0b11101111:
            angle = -2;
            break;
        case 0b11001111:
            angle = -3;
            break;
        case 0b11011111:
            angle = -5;
            break;
        case 0b10011111:
            angle = -7;
            break;
        case 0b10111111:
            angle = -9;
            break;
        case 0b00111111:
            angle = -10;
            break;
        case 0b01111111:
            angle = -12;
            break;
        default:
            angle = 0;  //default angle if there is no matching case
            break;
    }
    error = 0 - angle; //Error math
    u = 4*error; //Gain = 4
    AddSpeed(u,0); //Add speed to the motor and set mode to Angle

}

void MotorPath() {
    AutomaticLineFollow(2); //Go through outer lane twice
    MotorBrake(); //Brake
    SwitchLane(); //Switch to inner lane
    AutomaticLineFollow(2); //Go through inner lane once
    MotorBrake(); //Brake
    wait10ms(500); //Waits 5 seconds
    TurnRight180(); //Turn around
    AutomaticLineFollow(1); //Go through inner lane in other direction
    SwitchLane(); //Switch to outer lane
    AutomaticLineFollow(1); //Go through outer lane in other direction
    MotorBrake(); //Brake
    wait10ms(500); //Waits 5 seconds
}

void AutomaticLineFollow(int a) {
    int numberPassed = 0; // Declare and initialize the variable

    // Loop until 'numberPassed' is less than 'a'
    while (numberPassed < a) {
        MotorSpeed(); //Modify speed for robot in front if needed
        MotorAngle(); //Makes sure the robot follows the line
        numberPassed += DetectPLine();  //if detecting the passing line, increment amount of loops by one.
    }
}

void AddSpeed(int u, int speedOrAngle) {    //Adds speed to motors to respond to MotorSpeed or MotorAngle
    CCPR1L = (CCPR1L + u) > 1023 ? 1023 : ((CCPR1L + u) < 0 ? 0 : CCPR1L + u);  //compares CCPR1L+u with 0, if its greater, change CCPR1L to CCPR1L+u, otherwise equals 0. It then does the same check with 1023

    if (!speedOrAngle){ //if 0 is passed in (sets it to the angle)
        u = -u; //If angle, then added speed to one motor is converted to slow down for other motor
    }

    CCPR2L = (CCPR2L + u) > 1023 ? 1023 : ((CCPR2L + u) < 0 ? 0 : CCPR2L + u);  //compares CCPR2L+u with 0, if its greater, change CCPR2L to CCPR2L+u, otherwise equals 0. It then does the same check with 1023
}

void SwitchLane() {
    while (1) {
        TurnRight45(); // Turn right 45 degrees
        MotorForwards(); // Move forward
        wait10ms(50);   // Wait for 0.5 seconds
        unsigned char sensorData = ReadSensorArray();   // Read sensor array

        // Check if sensor data indicates the desired lane (example: all sensors are activated)
        if (sensorData != 0b1111111) {
            MotorBrake(); // Brake if lane is not detected
            break; // Exit the loop
        }
    }
}

int DetectPLine() {
    char arrayDetect = ReadSensorArray(); //Read the sensor array
    if (arrayDetect == 0b00000000){ //If the sensor detects all white (a line)
        return 1; //line detected
    } else { //If the sensor does not detect all white
        return 0; //line not detected
    }
}

void I2C_Initialise(void){      //Initialise I2C
  SSPCON1 = 0b00101000;     //set to master mode, enable SDA and SCL pins
  SSPCON2 = 0;                  //reset control register 2
  SSPADD = 0x63;                //set baud rate to 100KHz
  SSPSTAT = 0;                  //reset status register
  }

void I2C_checkbus_free(void){        //Wait until I2C bus is free, this is WaitI2C in the flowchart
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));    //wait until I2C bus is free
}

void I2C_Start(void){        //Generate I2C start condition
  I2C_checkbus_free();      //Test to see I2C bus is free
  SEN = 1;                  //Generate start condition,SSPCON2 bit 0 = 1
}

void I2C_RepeatedStart(void){     //Generate I2C Repeat start condition
  I2C_checkbus_free();          //Test to see I2C bus is free
  RSEN = 1;                     //Generate repeat start, SSPCON2 bit1 = 1
}

void I2C_Stop(void){         //Generate I2C stop condition
  I2C_checkbus_free();          //Test to see I2C bus is free
  PEN = 1;                      // Generate stop condition,SSPCON2 bit2 = 1
}

void I2C_Write(unsigned char write){     //Write to slave
  I2C_checkbus_free();          //check I2C bus is free
  SSPBUF = write;               //Send data to transmit buffer
}

unsigned char I2C_Read(void){    //Read from slave
  unsigned char temp;
  I2C_checkbus_free();      //Test to see I2C bus is free
  RCEN = 1;                 //enable receiver,SSPCON2 bit3 = 1
  I2C_checkbus_free();      //Test to see I2C bus is free
  temp = SSPBUF;            //Read slave
  I2C_checkbus_free();      //Test to see I2C bus is free
  ACKEN = 1;                //Acknowledge
  return temp;              //return sensor array data
}

unsigned char ReadSensorArray() {
    unsigned char linesensor;
    I2C_Start();                    //Send Start condition to slave
    I2C_Write(0x7C);                //Send 7 bit address + Write to slave
    I2C_Write(0x11);                //Write data, select RegdataA and send to slave
    I2C_RepeatedStart();            //Send repeat start condition
    I2C_Write(0x7D);                //Send 7 bit address + Read
    linesensor=I2C_Read();          //Send data to linesensor
    I2C_Stop();                     //Send Stop condition
    return linesensor;
}

