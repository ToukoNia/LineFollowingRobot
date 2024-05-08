/*
 * File:   main.c
 * Author: Group 14
 *
 * Created on 08 March 2024, 21:59

    This is code for the ACS11002 Project. It programs a shadowbot running on a PIC18F221 microcontroller.
    It programs the robot to go round a circular track with a white line on a black background.
    It follows these conditions:
        -It sets up and flashes the LEDs for 5 seconds
        -It goes round the outer loop twice
        -It changes lane
        -It goes round the inner loop once and then stops at the passing line
        -It waits 5 seconds
        -It turns back around and does the inner loop in reverse
        -Changes lane back to outer loop
        -Continues till it reaches the passing line, stops then blinks the LEDs for 5 seconds

    Whilst doing this, it automatically will stop if it detects a robot ahead.
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

#define ENC_TURN_VAL 20 //defines constant for the number of rotations that is required to turn 90 degrees
#define COLLISION_THRESH 300 //defines constant to begin to avoid collision advoidance
#define K 30    //defines constant for the angle gain
#define Ks 1.8    //defines constant for the speed gain. Maximum value was calculated from StartSpeed/(MAX_ERROR)
#define START_MARKSPACE 500    //sets the start markspace for the motors (this is also the value the motors reset to)
#define SETPOINT_DISTANCE 50    //sets the setpoint distance before automatic braking starts stopping the robot. 50 was picked as it allowed controlled stop and following of the robot in front without causing the robot to slow down if someone was standing too close to a corner of the track etc.

void Setup(void);
unsigned int ReadRADC(void);
void FlashLED(void);
void AllLED(unsigned int val);
void MotorForwards(void);
void TurnRight(void);
void TurnRight45(void);
void TurnRight180(void);
void EncoderChecker(int encVal);
void MotorBrake(void);
void Wait10ms(int del);
unsigned int MotorSpeed(void);
void MotorAngle(void);
void AddSpeed(int u, unsigned int speedOrAngle);
void SwitchLane(void);
int DetectPLine(void);
void I2C_Initialise(void);
void I2C_checkbus_free(void);
void I2C_Start(void);
void I2C_RepeatedStart(void);
void I2C_Stop(void);
void I2C_Write(unsigned char write);
unsigned char I2C_Read(void);
unsigned char ReadSensorArray(void);
void AutomaticLineFollow(int a);
void MotorPath(void);
void ResetMarkspace(void);
void ConfigPWM(void);

int markspaceL;    //Mark space ratio for Left motor
int markspaceR;    //Mark space ratio for Right motor

void main(void) {
    Setup();
    MotorPath();
    while(1);
}

void Setup(){
    TRISCbits.RC1=0;       //set CCP1(pin13) to an output pin
    TRISCbits.RC2=0;       //set CCP1(pin13) to an output pin
    TRISB=0b00000000;      //set Motor pins to be output pins
    TRISA=0b11001111;      //set Motor pins to be output pins
    ConfigPWM();            //Configure PWM

    ResetMarkspace();

    ADCON1=0b00001101;  //Set voltage reference and port A0 as analogue input
    ADCON2=0b10000010; // Fosc/32, A/D result right justified
    LATB=0;             //Turn all Leds off

    TRISCbits.RC3=1; //Set the RC3 pin as input
    TRISCbits.RC4=1; //Set the RC4 pin as output
    I2C_Initialise(); //Initialise the I2C

    FlashLED(); //Flash the LED after set up

    //Set up complete
    return;
}

void ConfigPWM(void){   //Configures PWM
    PR2 = 0b11111111 ;     //set period of PWM,610Hz
    T2CON = 0b00000111 ;   //Timer 2(TMR2)on, prescaler = 16
    CCP1CON = 0b00001100;   //enable CCP1 PWM
    CCP2CON = 0b00001100;   //enable CCP2 PWM
    CCPR1L = 0;             //turn left motor off
    CCPR2L = 0;             //turn Right motor off
    return;
}

void ResetMarkspace(){  //this sets both of the motor speeds back to their default values
    markspaceL=START_MARKSPACE;
    markspaceR=START_MARKSPACE;
}

void MotorPath() {
    AutomaticLineFollow(2); //Go through outer lane twice
    SwitchLane(); //Switch to inner lane
    AutomaticLineFollow(2); //Go through inner lane once
    MotorBrake(); //Brake
    Wait10ms(500); //Waits 5 seconds
    TurnRight180(); //Turn around
    AutomaticLineFollow(1); //Go through inner lane in other direction
    SwitchLane(); //Switch to outer lane
    AutomaticLineFollow(1); //Go through outer lane in other direction
    MotorBrake(); //Brake
    FlashLED(); //Flashes LEDs
    return;
}

void AutomaticLineFollow(int a) {
    unsigned int numberPassed = 0; // Declare and initialize the variable
    MotorAngle();
    while(ReadSensorArray()==0b00000000);   //lets it get over the line after turning 180
    // Loop until 'numberPassed' is less than 'a'
    while (numberPassed < a) {
        MotorAngle(); //Makes sure the robot follows the line
        numberPassed += DetectPLine();  //if detecting the passing line, increment amount of loops by one.
    }
    return;
}

void AllLED(unsigned int val){ //sets every LED to either on or off based on if one or zero entered.
    LED1=val;
    LED2=val;
    LED3=val;
    LED4=val;
    return;
}

void FlashLED(){       //flashes the LEDs on and off for 5 seconds
    for (unsigned int i=0;i<5;i++){
        AllLED(1);             //turn all LEDs on
        Wait10ms(50);          //wait 1 second
        AllLED(0);             //turn all LEDs off
        Wait10ms(50);          //wait 1 second
    }
    return;
}
void MotorForwards(){
    CCP1CON=0b00001100; //enables PWMA
    CCP2CON=0b00001100; //enables PWMB

    A1=0; //set A1 of motor A to 0 (low)
    A2=1; //set A2 of motor A to 1 (high)

    B1=0; //set B1 of motor B to 0 (low)
    B2=1; //set B2 of motor B to 1 (high)

    //manages updating speed in accordance to the values stored in markspaceL and R. only managed when checking MotorForwards() to increase control.
    CCP1CON = (0x0c)|((markspaceL&0x03)<<4);//
    CCPR1L = markspaceL>>2;
    CCP2CON = (0x0c)|((markspaceR&0x03)<<4);
    CCPR2L = markspaceR>>2;
    return;
}

void TurnRight(){
    CCP1CON=0b00001100; //enables PWMA
    CCP2CON=0b00001100; //enables PWMB

    A1=1; //set A1 of motor A to 1 (high)
    A2=0; //set A2 of motor A to 0 (low)
    //this sets the right wheel to reverse

    B1=0; //set B1 of motor B to 0 (low)
    B2=1; //set B2 of motor B to 1 (high)
    //this sets the left wheel to forwards

    return;
}

void TurnRight45(){
    TurnRight(); //Call the function to turn the robot right
    EncoderChecker(ENC_TURN_VAL); // Check the encoder values to ensure the robot turns approximately 45 degress
    return; //Exit the function
}



void TurnRight180(){
    TurnRight(); //Call the function to turn the robot right
   // EncoderChecker(4*ENC_TURN_VAL); //// Check the encoder values to ensure the robot turns approximately 180 degrees (4 times ENC_TURN_VAL)
    while(ReadSensorArray()!=0b11111111);   //while still on first line (detecting any white), wait
    // Check if sensor data indicates that the robot has detected the other line
    while(ReadSensorArray()==0b11111111);   //stop when at back on the+ line (sensor not detecting all black))
    MotorBrake();
    return; //Exit the function
}

void EncoderChecker(int encVal){
    unsigned int r,encCountL=0;
    unsigned int l,encCountR=0;

    while(encCountL<encVal || encCountR<encVal){  //loops until both constants are met
        if (!r){
            encCountR=encCountR+REnc;           //sums encoder values. The value is 1 when a turn is complete, and 0 otherwise. Hence, will only increment it if true
            if (encCountL >= encVal){       //Checks if left encoder count has reaches "encVal"
                B1=1; //If the left encoder count meets or exceeds "encVal", brake the left motor
                B2=1;
            }
        }
        r=REnc; //this make it such that it won't trigger until after it hasn't had subsequent values of ti having 1 to prevent issues.
        if (!l){
             encCountL=encCountL+LEnc;
              if (encCountR >= encVal){      //Checks if the right encoder count has reaches "encVal"
                    A1=1;  // If right encoder count meets or exceeds encVal, brake the right motor
                    A2=1;
            }
        }
        l=LEnc; //same as r
    };
    return;
}

void MotorBrake(){
    A1=1; //sets right motor pins to high
    A2=1; //sets right motor to brake

    B1=1; //sets left motor pins to high
    B2=1; //sets left motor to brake
    return; //Exits the function
}

void Wait10ms(int del){
    unsigned int c;
    for(c=0;c<del;c++)      //Loop to delay for 'del' multiples of 10 milliseconds
        __delay_ms(10);
    return;
}

unsigned int ReadRADC() {
    ADCON0 = 0b00000111; //select A/D channel AN0,start conversion
    while (ADCON0bits.GO){}; //do nothing while conversion in progress
    return ((ADRESH << 8) + ADRESL); //Combines high and low A/D bytes into one
}                                 // value and returns this A/D value 0-1023

unsigned int MotorSpeed(){
    unsigned int distance;    //variable to store distance
    int u;        //variable to store control response calculated from error between desired (0) and actual speed differences between the cars
    ResetMarkspace();
    distance=ReadRADC();
    if (distance>=COLLISION_THRESH){  //check if there are obstacles detected
         // Brake if obstacle detected
        return 1;
    } else if (distance>SETPOINT_DISTANCE){
        u=Ks*(SETPOINT_DISTANCE-distance);  //Calculate the control response using a proportional control strategy with a constant gain 'Ks' based on the distance difference from setpoint differences
        AddSpeed(u,1); //Adjust motor speed based on the control response, with both motors being altered equally
    }
    return 0;
}

void MotorAngle() {
    int angle;
    int error;
    int u;
    switch (ReadSensorArray()) {   // Angle is determined based on the sensor array data
        case 0b11111110:    //1 represents black, 0 represents white
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
            angle = 0;  //default angle if there is no matching case, means it is either going to be at the passing line or centred.
            break;
    }
    while(MotorSpeed()){    //loops until the obstacle is further away. This prevents errors with continuing to follow the line after MotorSpeed() was ran.
        MotorBrake();
    }
    error = 0 - angle; //Calculates the error between the desired angle and the actual angle

    if (error){
       u = K*error; //Gain = K
       AddSpeed(u,0); //Change the speeds on the motor and set mode to Angle
    }
    MotorForwards();

    return;
}


void AddSpeed(int u, unsigned int speedOrAngle) {    //Adds speed to motors to respond to MotorSpeed or MotorAngle

    markspaceL = (markspaceL + u) > 1023 ? 1023 : ((markspaceL + u) < 0 ? 0 : markspaceL + u);  //compares markspaceL+u with 0, if its greater, change markspaceL to markspaceL+u, otherwise equals 0. It then does the same check with 1023

    if (!speedOrAngle){
        u = -u; //If 0 (set to angle), then added speed to one motor is converted to the opposite direction for other motor (allows turning)
    }

    markspaceR = (markspaceR + u) > 1023 ? 1023 : ((markspaceR + u) < 0 ? 0 : markspaceR + u);  //compares markspaceR+u with 0, if its greater, change markspaceR to markspaceR+u, otherwise equals 0. It then does the same check with 1023

    return;
}


void SwitchLane() {
    ResetMarkspace();
    TurnRight45(); // Turn the robot right by approximately 45 degrees
    MotorForwards(); // Move forward
    while(ReadSensorArray()!=0b11111111);   //while still on first line (detecting any white), wait
    // Check if sensor data indicates that the robot has detected the other line
    while(ReadSensorArray()==0b11111111);   //stop when at second line (sensor not detecting all black))
    MotorBrake();
    return;
}

int DetectPLine() {
    if (ReadSensorArray() == 0b00000000){ //If the sensor detects all white (a line)
        while(ReadSensorArray()==0b00000000);   //loop till line no longer detected (stops errors of seeing counting the same line multiple times)
        return 1; //line detected
    } else { //If the sensor does not detect all white
        return 0; //return line not detected
    }
}

void I2C_Initialise(void){      //Initialise I2C
  SSPCON1 = 0b00101000;     //set to master mode, enable SDA and SCL pins
  SSPCON2 = 0;                  //reset control register 2
  SSPADD = 0x63;                //set baud rate to 100KHz
  SSPSTAT = 0;                  //reset status register
}

void I2C_checkbus_free(void){        //Wait until I2C bus is free
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F));
}

void I2C_Start(void){        //Generate I2C start condition
  I2C_checkbus_free();
  SEN = 1;                  //Generate start condition
}

void I2C_RepeatedStart(void){     //Generate I2C Repeat start condition
  I2C_checkbus_free();
  RSEN = 1;                     //Generate repeat start
}

void I2C_Stop(void){         //Generate I2C stop condition
  I2C_checkbus_free();
  PEN = 1;                      // Generate stop condition
}

void I2C_Write(unsigned char write){     //Write to slave
  I2C_checkbus_free();          //check I2C bus is free
  SSPBUF = write;               //Send data to transmit buffer
}

unsigned char I2C_Read(void){    //Read from slave
  unsigned char temp;
  I2C_checkbus_free();
  RCEN = 1;
  I2C_checkbus_free();
  temp = SSPBUF;
  I2C_checkbus_free();
  ACKEN = 1;
  return temp;              //return sensor array data
}

unsigned char ReadSensorArray() {   //reads the value from the sensor array and returns it in accordance to the I2C documentation
    unsigned char linesensor;
    I2C_Start();
    I2C_Write(0x7C);
    I2C_Write(0x11);
    I2C_RepeatedStart();
    I2C_Write(0x7D);
    linesensor=I2C_Read();
    I2C_Stop();
    return linesensor;
}
