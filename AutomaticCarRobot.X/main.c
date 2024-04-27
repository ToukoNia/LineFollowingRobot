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

#define ENCTURNVAL 9 //defines constant for the number of rotations that is required to turn 90 degrees
#define COLLISION_THRESH 300 //defines constant to begin to avoid collision advoidance
#define K 40
#define Ks 2.5
#define STARTMARKSPACE 500
#define SETPOINTDISTANCE 500

void Setup(void);
unsigned int readRADC(void);
void FlashLED(void);
void allLED(int val);
void MotorForwards(void);
void TurnRight(void);
void TurnRight45(void);
void TurnRight180(void);
void EncoderChecker(int encVal);
void MotorBrake(void);
void wait10ms(int del);
void MotorSpeed(void);
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
unsigned char ReadSensorArray();
void AutomaticLineFollow(int a);
void MotorPath(void);
void TurnLeft(void);
void TurnLeft45(void);
void ResetMarkspace(void);
void configPWM(void);
unsigned int markspaceL;    //Mark space ratio for Left motor
unsigned int markspaceR;    //Mark space ratio for Right motor

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
    configPWM();            //Configure PWM

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

void configPWM(void){   //Configures PWM
    PR2 = 0b11111111 ;     //set period of PWM,610Hz
    T2CON = 0b00000111 ;   //Timer 2(TMR2)on, prescaler = 16
    CCP1CON = 0b00001100;   //enable CCP1 PWM
    CCP2CON = 0b00001100;   //enable CCP2 PWM
    CCPR1L = 0;             //turn left motor off
    CCPR2L = 0;             //turn Right motor off
    return;
}

void ResetMarkspace(){
    markspaceL=STARTMARKSPACE;
    markspaceR=STARTMARKSPACE;
}

void MotorPath() {
    AutomaticLineFollow(2); //Go through outer lane twice
    SwitchLane(); //Switch to inner lane
    AutomaticLineFollow(2); //Go through inner lane once
    MotorBrake(); //Brake
    wait10ms(500); //Waits 5 seconds
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
    while(ReadSensorArray()==0b00000000);
    // Loop until 'numberPassed' is less than 'a'
    while (numberPassed < a) {
        MotorAngle(); //Makes sure the robot follows the line
        numberPassed += DetectPLine();  //if detecting the passing line, increment amount of loops by one.
    }
    return;
}

void FlashLED(){       //flashes the LEDs on and off for 5 seconds
    for (unsigned int i=0;i<1;i++){
        allLED(1);             //turn all LEDs on
        wait10ms(50);          //wait 1 second
        allLED(0);             //turn all LEDs off
        wait10ms(50);          //wait 1 second
    }
    return;
}

void allLED(int val){ //sets every LED to either on or off based on if one or zero entered.
    LED1=val; //sets LED1 to the value specified
    LED2=val; //sets LED2 to the value specified
    LED3=val; //sets LED3 to the value specified
    LED4=val; //sets LED4 to the value specified
    return;
}

void MotorForwards(){
    CCP1CON=0b00001100; //enables PWMA
    CCP2CON=0b00001100; //enables PWMB

    A1=0; //set A1 of motor A to 0 (low)
    A2=1; //set A2 of motor A to 1 (high)

    B1=0; //set B1 of motor B to 0 (low)
    B2=1; //set B2 of motor B to 1 (high)

    CCP1CON = (0x0c)|((markspaceL&0x03)<<4);//0x0c enables PWM,then insert the 2 LSB
    CCPR1L = markspaceL>>2; //of markspaceL into CCP1CON and the higher 8 bits into
    CCP2CON = (0x0c)|((markspaceR&0x03)<<4); //markspaceL.  Same as above but for
    CCPR2L = markspaceR>>2;                  // CCP2CON and CCPR2L
    return;
}

void TurnLeft(){
    CCP1CON=0b00001100; //enables PWMA
    CCP2CON=0b00001100; //enables PWMB

    A1=0;
    A2=1;
    //this sets the right wheel to forward

    B1=1;
    B2=0;
    //this sets the left wheel to reverse

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
   // EncoderChecker(ENCTURNVAL); // Check the encoder values to ensure the robot turns approximately 45 degress
    wait10ms(25);
    MotorBrake();
    return; //Exit the function
}
void TurnLeft45(){
    TurnLeft(); //Call the function to turn the robot right
    EncoderChecker(ENCTURNVAL); // Check the encoder values to ensure the robot turns approximately 45 degress
    return; //Exit the function
}


void TurnRight180(){
    TurnRight(); //Call the function to turn the robot right
   // EncoderChecker(4*ENCTURNVAL); //// Check the encoder values to ensure the robot turns approximately 180 degrees (4 times ENCTURNVAL)
    while(ReadSensorArray()!=0b11111111);   //while still on first line (detecting any white), wait
    // Check if sensor data indicates that the robot has detected the other line
    while(ReadSensorArray()==0b11111111);   //stop when at back on the+ line (sensor not detecting all black))
    MotorBrake();
    return; //Exit the function
}

void EncoderChecker(int encVal){
    unsigned int encCountL=0; //Initialize the left encoder count
    unsigned int encCountR=0; //Initialize the right encoder count
    while(encCountL<encVal || encCountR<encVal){  //loops until both constants are met
        encCountR+=REnc;           //sums encoder values. The value is 1 when a turn is complete, and 0 otherwise. Hence, will only increment it if true
        encCountL+=LEnc;
        if (encCountL >= encVal){       //Checks if left encoder count has reaches "encVal"
            B1=1; //If the left encoder count meets or exceeds "encVal", brake the left motor
            B2=1;
        }
        if (encCountR >= encVal){      //Checks if the right encoder count has reaches "encVal"
            A1=1;  // If right encoder count meets or exceeds encVal, brake the right motor
            A2=1;
        }
        while(REnc&&LEnc);
        while(REnc);
        while(LEnc);
    };
    return;

}

void MotorBrake(){
    A1=1; //sets left motor pins to high
    A2=1; //sets right motor to brake

    B1=1; //sets left motor pins to high
    B2=1; //sets left motor to brake
    return; //Exits the function
}


unsigned int readRADC() {
    ADCON0 = 0b00000111; //select A/D channel AN0,start conversion
    while (ADCON0bits.GO){}; //do nothing while conversion in progress
    return ((ADRESH << 8) + ADRESL); //Combines high and low A/D bytes into one
}                                 // value and returns this A/D value 0-1023
/*
unsigned int readLADC() {
    ADCON0 = 0b00000011; //select A/D channel AN0,start conversion
    while (ADCON0bits.GO){}; //do nothing while conversion in progress
    return ((ADRESH << 8) + ADRESL); //Combines high and low A/D bytes into one
}                                 // value and returns this A/D value 0-1023
*/

void wait10ms(int del){
    unsigned int c;         //Declare a variable to use as a counter
    for(c=0;c<del;c++)      //Loop to delay for 'del' multiples of 10 milliseconds
        __delay_ms(10);     //Call a function to delay for 10 milliseconds
    return;              //Exit the function
}

void MotorSpeed(){

    unsigned int distance;    //variable to store distance
    int u;        //variable to store control response calculated from error between desired (0) and actual speed differences between the cars
    ResetMarkspace();
    distance=readRADC();
    if (distance>=COLLISION_THRESH){  //check if there are obstacles detected
        MotorBrake();  // Brake if obstacle detected
    } else{
        MotorForwards();  //Move forward if no obstacle detected
        u=Ks*(SETPOINTDISTANCE-distance);  //Calculate the control response using a proportional control strategy with a constant gain 'Ks' based on the distance difference from setpoint differences
        AddSpeed(u,1); //Adjust motor speed based on the control response, with both motors being altered equally
    }
    return;
}

void MotorAngle() {
    int angle; //variable to store angle
    int error; //variable to store error
    int u;     //variable to store control response
    unsigned char colourArray = ReadSensorArray(); //Read array data from the colour sensor
    MotorSpeed();
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
        case 0b11111111:    //if all black, brake.
            MotorBrake();
            break;
        default:
            angle = 0;  //default angle if there is no matching case, means it is either going to be at the passing line or centred.
            break;
    }
    error = 0 - angle; //Calculates the error between the desired angle and the actual angle
    u = K*error; //Gain = K
    if (error){
       AddSpeed(u,0); //Change the speeds on the motor and set mode to Angle
    }
    MotorForwards();
    return;
}



void AddSpeed(int u, unsigned int speedOrAngle) {    //Adds speed to motors to respond to MotorSpeed or MotorAngle
    //markspaceL controls the right motor speed and vice versa

    markspaceL = (markspaceL + u) > 1023 ? 1023 : ((markspaceL + u) < 0 ? 0 : markspaceL + u);  //compares markspaceL+u with 0, if its greater, change markspaceL to markspaceL+u, otherwise equals 0. It then does the same check with 1023

    if (!speedOrAngle){ //if 0 is passed in (sets it to the angle)
        u = -u; //If angle, then added speed to one motor is converted to slow down for other motor
    }

    markspaceR = (markspaceR + u) > 1023 ? 1023 : ((markspaceR + u) < 0 ? 0 : markspaceR + u);  //compares markspaceR+u with 0, if its greater, change markspaceR to markspaceR+u, otherwise equals 0. It then does the same check with 1023

    return;
}

void SwitchLane() {
    ResetMarkspace();   //resets any angle to being back to straight. It also resets the speed
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
        while(ReadSensorArray()==0b00000000);
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
