
/*******************************************************************************
Module:
  mainLineFollower.c

Author - Shubham Aggarwal
Last Modified Date - Jan 19th, 2013

Overview:
 This program is the codebase for a linefollowing robot that was created to
 participate in the Robots Get Grimm CETA Robotics Competition. It allows the
 robot to complete a set of three challenges, which are defined in their
 respective function descriptions. The main function of the robot and program
 was to follow a black line on a white board as well as carry an aluminum pail.
 The robot that was created had four line following sensors; two in the front of
 the robot and two in the back. This was to allow the robot to follow the line
 both forwards and backwards. The mechanism to carry the pail was an intake
 system that consisted of two rollers, each individually powered. The intake
 "sucked in" the pail onto to a plate. There was a thermistor used to detect
 the temperature of the pail for Challenge Three and a pushbutton to detect the
 the pail in the intake system. The Microcontroller used to control the robot
 was the PIC24FV32KA302 MCU by Microchip.

Hardware Notes:
 Front Left Line Following Sensor       RB4/AN15
 Front Right Line Following Sensor      RB2/AN4
 Back Left Line Following Sensor        RA2/AN13
 Back Right Line Following Sensor       RA1/AN1

 Front Left LED Indicator               RB6
 Front Right LED Indicator              RB1/AN3
 Back Left LED Indicator                RB8
 Back Right LED Indicator               RB0/AN2

 Potentiometer                          RB3/AN5
 Intake Button                          RB9
 Thermistor                             RA0/AN0

 Drive System H-Bridge (SN754410NE IC)
     1-2EN                              RB11/OC2
     3-4EN                              RB10/OC3
     1A (Left Drive Motor)              RB13/AN11
     2A (Left Drive Motor)              RB12/AN12
     3A (Right Drive Motor)             RB15/AN9
     4A (Right Drive Motor)             RB14/AN10

 Intake System H-Bridge (SN754410NE IC)
     1-2EN                              +6V
     3-4EN                              +6V
     1A                                 RB5
     2A                                 RA4
     3A                                 Wired to 2A
     4A                                 Wired to 1A

*******************************************************************************/

/*** Include Files ************************************************************/

#include "p24fv32ka302.h"
#include "configBits.h"

/*** Symbolic Constants used by main() ****************************************/

//Indicator LEDs used with the line following sensors.
#define FRONT_RIGHT_DETECT      _LATB1
#define FRONT_LEFT_DETECT       _LATB6
#define BACK_RIGHT_DETECT       _LATB0
#define BACK_LEFT_DETECT        _LATB8

//Used to set the PWM values for the drive motors.
#define LEFT_DRIVE_PWM          OC2R
#define RIGHT_DRIVE_PWM         OC3R

//Calibrated values that are used in the sensorCalibration function. They are
//slightly larger than the black value for the front and back sensors.
#define FRONT_LINE_THRESHOLD     150
#define BACK_LINE_THRESHOLD      170

//Used to define the path the robot takes during Challenge Three.
#define LEFT                     1
#define STRAIGHT                 2
#define RIGHT                    3

//Used with the line sensor indicator LEDs
#define ON                       1
#define OFF                      0

/*** Local Function Prototypes ************************************************/

void initialize(void);       //Initialize PIC24 MCU

/*These four functions, used together, form a basic control loop for a line
 *following robot. This control loop was used during Challenge One but not
 *for Challenge Two or Three. However, some of these functions were used for the
 *other Challenges */
void getInputs(void);        //Read sensor and potentiometer inputs
void updateIndicators(void); //Refresh Indicators with line sensor states
void setOutputs(void);       //Update motors based on current robot state
void timing(void);           //Determines how fast the control loop executes

/*These three functions contain the code for their respective challenges. They
 *make use of the rest of the functions in the code to allow the robot to 
 *complete the tasks required.*/
void challengeOne (void);
void challengeTwo(void);
void challengeThree(void);

/*Calibrates the line following sensors live using the threshold values
 *defined as constants.*/
void sensorCalibration(void); 

/*These functions set the individual motor ports to a 1 or 0, depending on the
 *parameters.*/
void setDriveMotors(int oneA, int twoA, int threeA, int fourA);
void setIntakeMotors(int leftMotor, int rightMotor);

/*These functions control the robot to follow the black line, either forwards
 *or backwards (depending on the function used) at a desired speed from 0-1023*/
void forwardLineFollow(int speed);
void backwardLineFollow(int speed);

/*These functions drive the robot, either forwards or backwards, at a desired
 *speed for a desired time.*/
void driveForward (int speed, int time);
void driveBackward (int speed, int time);

/*These functions allow the robot to turn left or right at a desired speed.
 *The first two are used for following the line. The other two are used to
 *perform sensor-based 90 degree or 180 degree turns, for Challenge Two and
 *Three.*/
void turnLeft(int speed); //
void turnRight(int speed);
void turnLeftUntilLine(int speed);
void turnRightUntilLine(int speed);

/*These functions allow the robot to get past a junction so that normal line
 *line following can resume.*/
void clearLineForward(void);
void clearLineBackward(void);

/*These functions detect when the robot has come to a junction and handle robot
 *behaviour accordingly.*/
void frontCounter(void);
void backCounter(void);

/*These functions control the intake sub-system. They allow the robot to intake
 *or outtake the pail.*/
void driveWithIntake();
void outtake(int time);

/*This function detects the temperature of the pail using the thermistor and
 *defines the path the robot must take to drop it off, during Challenge Three*/
void senseTemperature(void);

/*Used throughout the code for time-based delays.*/
void delayMs(unsigned int ms); 

/*** Global Variable Declarations *********************************************/

/*These variables store the line sensor values*/
unsigned int frontLeftOpto;     //(RB4/AN15): Front Left Line Sensor
unsigned int frontRightOpto;    //(RB2/AN4): Front Right Line Sensor
unsigned int backLeftOpto;      //(RA2/AN13): Back Left Line Sensor
unsigned int backRightOpto;     //(RA1/AN1): Back Right Line Sensor

/*These variables store the individual sensor thresholds that are used to detect
 *whether the sensor is seeing white or black.*/
unsigned int frontLeftThreshold = 0;    
unsigned int frontRightThreshold = 0;
unsigned int backLeftThreshold = 0;
unsigned int backRightThreshold = 0;

unsigned int thermistor;                //Stores the thermistor reading

/*These variables are counters that are used to keep track of various things*/
int lineCounter = 0;           //Keeps track of the number of junctions detected
int lapCounter = 0;            //Keeps track of the laps the robot completes
int loopCounter = 0;           /*Used to detect a "time-out" situation for the
                                *intake. Counts the number of cycles the
                                *driveWithIntake function executes.*/

int robotState = 0; /*Keeps track of the state that the robot is in and
                     *controls the sequencing for the Challenges.*/
int button = 0;     //Stores the intake switch state
int path = 0;       //Indicates the path that the robot must take in Challenge 3
unsigned int challengeSelect = 0; /*Stores the value of the Challenge Selection
                                   *potentiometer.*/

/*** main() Function **********************************************************/

int main(void)
{
    initialize();           //Initialize the PIC
    getInputs();            //Read the initial sensor values
    sensorCalibration();    //Calibrate line sensors

    //Run the selected Challenge, based off of the potentiometer value
    switch (challengeSelect)
    {
        case 0 ... 330:
            challengeOne();
            break;
        case 331 ... 666:
            challengeTwo();
            break;
        case 667 ... 1023:
            challengeThree();
            break;
    }
}

/*******************************************************************************
 * Function:        void initialize(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes the microcontroller and the peripherals
 *                  used. H-bridge, sensors and other  I/O are configured.
 ******************************************************************************/

void initialize(void)
{
   //Initialize Timer1 for main loop delay of 3.2mS
   //(Fcyc/2 = 4 MHz)
    T1CONbits.TCS = 0;     //Set T1 Clk Source = Fosc/2
    T1CONbits.TCKPS = 3;   //Set T1 Clk Pre-scale = 1:256
                           //(TMR1 CLK = 15.625kHz)
    PR1 = 50;              //Set period = ~3mS
    TMR1 = 0;              //Clear T1 counter
    IFS0bits.T1IF = 0;     //Clear T1 overflow/interrupt flag
    T1CONbits.TON = 1;     //Start T1

   /*********************LINE FOLLOWING LED INDICATORS*************************/

   //Front Right Sensor LED Indicator (RB1)
    _LATB1 = 0;
    _TRISB1 = 0;   //Make the pin a digital output

   //Front Left Sensor LED Indicator (RB1)
    _LATB6 = 0;
    _TRISB6 = 0;  //Make the pin a digital output

   //Back Right Sensor LED Indicator (RB0)
    _LATB0 = 0;
    _TRISB0 = 0;  //Make the pin a digital output

   //Back Left Sensor LED Indicator (RB8)
    _LATB8 = 0;
    _TRISB8 = 0;  //Make the pin a digital output

   /******************************INTAKE BUTTON********************************/
   
   //Intake Button, used to detect pail (RB9)
    _LATB9 = 0;
    _TRISB9 = 1;  //Make the pin a digital input

   /****************************INTAKE H-BRIDGE********************************/
    
   //Intake system H-Bridge 1A (RB5)
    _LATB5 = 0;
    _TRISB5 = 0;  //Make the pin a digital output

   //Intake system H-Bridge 2A (RA4)
    _LATA4 = 0;
    _TRISA4 = 0;  //Make the pin a digital output

   /**********************DIGITAL TO ANALOG CONVERSIONS************************/
    
   //Convert line sensor, potentiometer and thermistor ports from digital to
   //analog

   //Front Left Line Sensor
    _ANSB4 = 1;     //Make RB4/AN15 input Analog

   //Back Left Line Sensor
    _ANSA2 = 1;     //Make RA2/AN13 input Analog

   //Front Right Line Sensor
    _ANSB2 = 1;     //Make RB2/AN4 input Analog

   //Back Right Line Sensor
    _ANSA1 = 1;     //Make RA1/AN1 input Analog

   //Thermistor
    _ANSA0 = 1;     //Make RA0/AN0 input Analog

   //Challenge Selection Potentiometer
    _ANSB3 = 1;     //Make RB3/AN5 input Analog

   /****************************ADC CONFIGURATION*****************************/
    
    AD1CON1 = 0x0000;        //SSRC = 0000 clearing SAMP starts conversion
                             //10-bit ADC mode selected
    AD1CHS = 0x0005;         //Connect RA0/AN0 as CH0 input
    AD1CSSL = 0;
    AD1CON3 = 0x1F02;        //Sample time = 31Tad, Tad = internal (2*Tcy)
    AD1CON2 = 0;

    _ADON = 1;               //Turn on ADC

   /*********************DRIVE MOTOR H-BRIDGE CONFIGURATION********************/

   //Initialize ports to drive "3-4EN" PWM signal to the drive motors' H-Bridge

   //Initialize "3-4EN" PWM control output (RB10/OC3)
    _TRISB10 = 0;     //Make the pin a digital output
    _LATB10 = 0;

   //Initialize "3A" control output (RB15/AN9)
    _TRISB15 = 0;     //Make the pin a digital output
    _LATB15 = 0;

   //Initialize "4A" control output (RB14/AN10)
    _TRISB14 = 0;    //Make the pin a digital output
    _LATB14 = 1;

   //Initialize Output Compare 3 (OC3) to drive Motor A PWM signal to "3-4EN"
   //We want to create 61.04Hz PWM frequency @ 1024 bits resolution (0-1023)
    OC3CON1bits.OCM = 0b000;       //Disable the OC module
    OC3R = 0;                      //Write the duty cycle for the 1st PWM pulse
    OC3CON1bits.OCTSEL = 0;        //Select Timer 2 as the OC time base
    OC3CON1bits.OCM = 0b110;       //Select the OC mode (Edge PWM)

   //Initialize ports to drive "1-2EN" PWM signal to the drive motors' H-Bridge

   //Initialize "1-2EN" PWM control output (RB11/OC2)
    _TRISB11 = 0;    //Make the pin a digital output
    _LATB11 = 0;

   //Initialize "1A" control output (RB13/AN11)
    _TRISB13 = 0;    //Make the pin a digital output
    _LATB13 = 0;

   //Initialize "2A" control output (RB12/AN12)
    _TRISB12= 0;     //Make the pin a digital output
    _LATB12 = 1;

   //Initialize Output Compare 2 (OC2) to drive Motor B PWM signal to "1-2EN"
   //We want to create 61.04Hz PWM frequency @ 1024 bits resolution (0-1023)
    OC2CON1bits.OCM = 0b000;      //Disable the OC module
    OC2R = 0;                     //Write the duty cycle for the 1st PWM pulse
    OC2CON1bits.OCTSEL = 0;       //Select Timer 2 as the OC time base
    OC2CON1bits.OCM = 0b110;      //Select the OC mode (Edge PWM)

   //Initialize and enable Timer 2 to create a 61.04Hz PWM frequency for both 
   //PWM channels
    T2CONbits.TON = 0;            //Disable Timer
    T2CONbits.TCS = 0;            //Select internal instruction clock
    T2CONbits.TGATE = 0;          //Disable Gated Timer Mode
    T2CONbits.TCKPS = 0b10;       //Select 1:64 prescale (57.578kHz)
    TMR2 = 0x00;                  //Clear timer register
    PR2 = 1024;                   //Load the period register

    IFS0bits.T2IF = 0;            //Clear Timer 2 interrupt flag
    T2CONbits.TON = 1;            //Start timer (starts PWMs)
}

/*******************************************************************************
 * Function:        void getInputs(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Obtains any input information either on-chip
 *                  (from internal registers, etc...) or off-chip
 *                  (pin voltage levels). Uses this information to modify
 *                  or update special data structures used in the control
 *                  function ()"
 *
 * Note:            None
 ******************************************************************************/
void getInputs(void)
{
   //Read and store the value of the Challenge Select Potentiometer
    AD1CHS = 0x0005;              //Connect RB3/AN5 as CH0 input
    _SAMP = 1;                    //Sample potentiometer value
    delayMs(5);                   //After 5mS start conversion
    _SAMP = 0;                    //Convert potentiometer value
    while(!_DONE);                //Conversion done? (takes 12*Tad)
    challengeSelect = ADC1BUF0;   //Yes, then save ADC value

   //Read and store the line sensor values
   //Front Right Sensor(RB2/AN4)
    AD1CHS = 0x0004;              //Connect RB2/AN4 as CH0 input
    _SAMP = 1;
    delayMs(5);
    _SAMP = 0;
    while(!_DONE);
    frontRightOpto = ADC1BUF0;

   //Front Left Sensor (RB4/AN15)
    AD1CHS = 0x000F;              //Connect RB4/AN15 as CH0 input
    _SAMP = 1;
    delayMs(5);
    _SAMP = 0;
    while(!_DONE);
    frontLeftOpto = ADC1BUF0;

   //Back Right Sensor (RA1/AN1)
    AD1CHS = 0x0001;              //Connect RA1/AN1 as CH0 input
    _SAMP = 1;
    delayMs(5);
    _SAMP = 0;
    while(!_DONE);
    backRightOpto = ADC1BUF0;

   //Back Left Sensor (RA2/AN13)
    AD1CHS = 0x000D;              //Connect RA2/AN13 as CH0 input
    _SAMP = 1;
    delayMs(5);
    _SAMP = 0;
    while(!_DONE);
    backLeftOpto = ADC1BUF0;

   //Thermistor (RA0/AN0)
    AD1CHS = 0x0000;              //Connect RA0/AN0 as CH0 input
    _SAMP = 1;
    delayMs(5);
    _SAMP = 0;
    while(!_DONE);
    thermistor = ADC1BUF0;
}

/*******************************************************************************
 * Function:        void updateIndicators(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Turns on each indicator LED based on the state of it's
 *                  corresponding sensor. The LED turns on when the sensor
 *                  detects the line and off when it detects white.
 *
 * Note:            None
 ******************************************************************************/
void updateIndicators(void)
{
    if(frontRightOpto < frontRightThreshold) //Front Right sensor detects line
    {
        FRONT_RIGHT_DETECT = ON; //Turn on indicator LED
    }
    else
    {
        FRONT_RIGHT_DETECT = OFF; //Turn off indicator LED
    }

    if(frontLeftOpto < frontLeftThreshold)   //Front Left Sensor detects line
    {
        FRONT_LEFT_DETECT = ON;
    }
    else
    {
        FRONT_LEFT_DETECT = OFF;
    }

    if(backLeftOpto < backLeftThreshold)     //Back Left Sensor detects line
    {
        BACK_LEFT_DETECT = ON;
    }
    else
    {
        BACK_LEFT_DETECT = OFF;
    }

    if(backRightOpto < backRightThreshold) //Back Right Sensor detects line
    {
        BACK_RIGHT_DETECT = ON;
    }
    else
    {
        BACK_RIGHT_DETECT = OFF;
    }
}

/*******************************************************************************
 * Function:        void setOutputs(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Based on the number of times the junction has been detected
 *                  (lineCounter variable), the robot will follow the line
 *                  forwards or backwards. After detecting the junction 4 times
 *                  (2 laps), the robot stops. 
 *
 * Note:            None
 ******************************************************************************/
void setOutputs(void)
{
   /*Robot will follow the line forwards at the beginning and the start of the
    *second lap*/
    if (lineCounter == 0 || lineCounter == 2)
    {
        forwardLineFollow(1000);
    }
    
   //Robot will follow the line backwards in the second half of both laps
    else if (lineCounter == 1 || lineCounter == 3)
    {
        backwardLineFollow(1000);
    }

   //Stop robot at the end of 2 laps
    else if (lineCounter == 4)
    {
        LEFT_DRIVE_PWM = 0;
        RIGHT_DRIVE_PWM = 0;
        setDriveMotors(0,0,0,0);
    }
}

/*******************************************************************************
 * Function:        void timing(void)
 *
 * PreCondition:    Timer1 initialized to provide the appropriate delay
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    No task in the main loop can block the CPU
 *                  Requires cooperative multi-tasking design
 *
 * Overview:        This function determines how fast the control loop runs
 *
 * Note:            None
 ******************************************************************************/
void timing(void)
{
   //Wait for T1 overflow
   while(!IFS0bits.T1IF);

   //Acknowledge and reset the flag
   IFS0bits.T1IF = 0;
}

/*******************************************************************************
 * Function:        void challengeOne(void)
 *
 * PreCondition:    Potentiometer must be between 0 and 330
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function makes use of the system control loop used in
 *                  previous projects. It reads the sensors, checks whether or
 *                  not the robot is at a junction, updates the LEDs and
 *                  controls the drive motors.
 *
 * Note:            None
 ******************************************************************************/
void challengeOne (void)
{
    clearLineForward(); //Moves the robot past the starting line junction
    while(1)
    {
        getInputs();        //Read sensors
        if (lineCounter == 0 || lineCounter == 2)
        {
           //Checks whether the front sensors have detected a junction
            frontCounter();
        }

        else if (lineCounter == 1 || lineCounter == 3)
        {
           //Checks whether the back sensors have detected a junction
            backCounter();
        }
        updateIndicators(); //Update LEDs
        setOutputs();
        timing();
    }
}

/*******************************************************************************
 * Function:        void challengeTwo(void)
 *
 * PreCondition:    Potentiometer must be between 331 and 666
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function uses a state-machine style control scheme to
 *                  allow the robot to complete Challenge Two. The robotState
 *                  variable controls the sequencing of the robot actions and is
 *                  incremented after each action is complete. This ensures that
 *                  the code is robust and the robot progresses through the
 *                  challenge as expected. Challenge Two requires the robot to
 *                  follow the line to the other end, pick up the pail, follow
 *                  the line back to the starting line and drop it off. It must
 *                  complete this twice.
 *
 * Note:            None
 ******************************************************************************/
void challengeTwo(void)
{
   clearLineForward(); //Moves the robot past the starting line junction
   while(1)
   {
        getInputs();                    //Read sensors
        updateIndicators();             //Update LEDs

        if (robotState == 0)
        {
            forwardLineFollow(985);
            //Robot reaches junction at other end of track
            if ((frontLeftOpto < frontLeftThreshold) &&
                    (frontRightOpto < frontRightThreshold))
            {
                robotState = 1;
            }
        }

        else if (robotState == 1)
        {
            driveWithIntake();          //Robot picks up pail
        }

        else if (robotState == 2)
        {
            backwardLineFollow(1023);   //Robot follows the line backwards
                                        //until starting line junction
            backCounter();
        }

        else if (robotState == 3)
        {
            turnRightUntilLine(1023);   //Performs a 180 degree turn right
        }

        else if (robotState == 4)
        {
            outtake(1000);              //Outtakes the pail
        }

        else if (robotState == 5)
        {
            turnLeftUntilLine(1023);    //Performs a 180 degree turn left
        }

        else
        {
            lapCounter++;
            robotState = 0;             //Code returns to first state structure
            //Completes this sequence of events twice then ends
             if (lapCounter == 2)
             {
                 break;
             }
        }
   }
}

/*******************************************************************************
 * Function:        void challengeThree(void)
 *
 * PreCondition:    Potentiometer must be between 667 and 1023
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function uses the state-machine style control scheme as
 *                  in Challenge Two to allow the robot to complete Challenge
 *                  Three. The robotState variable controls the sequencing of
 *                  the robot actions and is incremented after each action is
 *                  complete. Challenge Three requires the robot to pick up the
 *                  pail, sense its temperature and drop it off at the correct
 *                  location ("Too Cold", "Just Right" or "Too Hot").
 *
 * Note:            None
 ******************************************************************************/
void challengeThree(void)
{
   clearLineForward();  //Moves the robot past the starting line junction
   while(1)
   {
       getInputs();                 //Read sensors
       updateIndicators();          //Update LEDs

       if (robotState == 0)
        {
            forwardLineFollow(800);
            //Robot reaches junction at other end of track
            if ((frontLeftOpto < frontLeftThreshold) &&
                    (frontRightOpto < frontRightThreshold))
            {
                LEFT_DRIVE_PWM = 0;
                RIGHT_DRIVE_PWM = 0;
                setDriveMotors(0,0,0,0);
                robotState = 1;
            }
        }

        else if (robotState == 1)
        {
            driveWithIntake();      //Robot intakes pail
            robotState = 2;
        }

       else if (robotState == 2)
       {
           senseTemperature();      //The temperature of the pail is sensed
           robotState = 3;
       }

       else if (robotState == 3)
       {
          //Pail is "Too Cold" so robot must turn left
           if (path == LEFT)
           {
               /*Front Left sensor detects line so turnLeftUntilLine won't work
                *and time-based turn must be used*/
               if (frontLeftOpto < frontLeftThreshold)
               {
                 turnLeft(1023);
                 delayMs(900);
                 setDriveMotors(0,0,0,0);
                 robotState = 4;
               }

               /*Front Left Sensor doesn't detect line so sensor based turning
                *can be used*/
               else 
               {
                   turnLeftUntilLine(1023);
                   robotState = 4;
               }
           }

          //Pail is "Just Right" (Room Temperature) so robot must go straight
           else if (path == STRAIGHT)
           {
               robotState = 4;
           }

          //Pail is "Too Hot" so robot must turn right
           else if (path == RIGHT)
           {
               /*Front Right sensor detects line so turnRightUntilLine won't
                *work and time-based turn must be used*/
               if (frontRightOpto < frontRightThreshold)
               {
                 turnRight(1023);
                 delayMs(900);
                 setDriveMotors(0,0,0,0);
                 robotState = 4;
               }

               /*Front Right Sensor doesn't detect line so sensor based turning
                *can be used*/
               else
               {
                   turnRightUntilLine(1023);
                   robotState = 4;
               }
           }
       }

       else if (robotState == 4)
       {
           forwardLineFollow(900);  //Follow the line to the dropoff square
           if ((frontLeftOpto < frontLeftThreshold) &&
                   (frontRightOpto < frontRightThreshold))
            {
               LEFT_DRIVE_PWM = 0;
               RIGHT_DRIVE_PWM = 0;
               setDriveMotors(0,0,0,0);
               robotState = 5;
            }
       }

       else if (robotState == 5)
       {
          /*Clear the junction at the dropoff square before outtaking, because
           *intake is long and pail would not land in square otherwise*/
           clearLineBackward();
           outtake(1000);           //Outtake pail
           robotState = 6;
       }

       else if (robotState == 6)
       {
           backwardLineFollow(900); //Follow line backwards to center junction
           if ((backLeftOpto < backLeftThreshold) &&
                   (backRightOpto < backRightThreshold))
           {
               LEFT_DRIVE_PWM = 0;
               RIGHT_DRIVE_PWM = 0;
               setDriveMotors(0,0,0,0);
               robotState = 7;
           }
       }

       else if (robotState == 7)
       {
          //Continue driving backwards a little to center the wheels on the line
           clearLineBackward();
           robotState = 8;
       }

       else if (robotState == 8)
       {
          /*Depending on the dropoff square the robot went to, the robot must
           *behave differently in order to get back to home square*/
           if (path == LEFT)
           {
               turnRight(1023);
               delayMs(850);
               setDriveMotors(0,0,0,0);
               robotState = 9;
           }

           else if (path == STRAIGHT)
           {
               robotState = 9;
           }

           else if (path == RIGHT)
           {
               turnLeft(1023);
               delayMs(850);
               setDriveMotors(0,0,0,0);
               robotState = 9;
           }
       }
       else if (robotState == 9)
       {
          //Follow line backwards to front edge of home square
           backwardLineFollow(900);
           if ((backLeftOpto < backLeftThreshold) &&
                   (backRightOpto < backRightThreshold))
           {
               LEFT_DRIVE_PWM = 0;
               RIGHT_DRIVE_PWM = 0;
               setDriveMotors(0,0,0,0);
               robotState = 10;
           }
       }

       else if (robotState == 10)
       {
          //Drive backwards a little to clear the front edge of the home square
           driveBackward (600, 300);
           robotState = 11;
       }

       else if (robotState == 11)
       {
          /*Continue driving backwards until back sensors detect back edge of
           *home square*/
           backwardLineFollow(900);
           if ((backLeftOpto < backLeftThreshold) &&
                   (backRightOpto < backRightThreshold))
           {
               LEFT_DRIVE_PWM = 0;
               RIGHT_DRIVE_PWM = 0;
               setDriveMotors(0,0,0,0);
               robotState = 12;
           }
       }

       else if (robotState == 12)
       {
          //Drive forwards just a little to put back of robot in home square
           driveForward(1023, 150);
           robotState++;
           break; //Exit loop
       }
   }
   while(1);
}

/*******************************************************************************
 * Function:        void sensorCalibration(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function calibrates all the sensors everytime the robot
 *                  is turned on. The robot is placed on the field behind the
 *                  starting line, with all sensors seeing white. The
 *                  calibration procedure takes the current sensor values and
 *                  subtracts the defined line threshold values. This new value
 *                  becomes the sensor threshold that is used to detect if the
 *                  sensor is on the line or not.
 *
 * Note:            None
 ******************************************************************************/
void sensorCalibration(void)
{
    frontLeftThreshold = frontLeftOpto - FRONT_LINE_THRESHOLD;
    frontRightThreshold = frontRightOpto - FRONT_LINE_THRESHOLD;
    backLeftThreshold = backLeftOpto - BACK_LINE_THRESHOLD;
    backRightThreshold = backRightOpto - BACK_LINE_THRESHOLD;
}

/*******************************************************************************
 * Function:      void setDriveMotors(int oneA, int twoA, int threeA, int fourA)
 *
 * PreCondition:    None
 *
 * Input:           int oneA (1A control input)
 *                  int twoA (2A control input)
 *                  int threeA (3A control input)
 *                  int fourA (4A control input)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function allows all four drive motor control inputs
 *                  to be accessed in one line. This makes it easier to set
 *                  the motor directions.
 *
 * Note:            None
 ******************************************************************************/
void setDriveMotors(int oneA, int twoA, int threeA, int fourA)
{
    _LATB13= oneA;
    _LATB12= twoA;
    _LATB15= threeA;
    _LATB14= fourA;
}

/*******************************************************************************
 * Function:        void setIntakeMotors(int leftMotor, int rightMotor)
 *
 * PreCondition:    None
 *
 * Input:           int leftMotor
 *                  int rightMotor
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function allows the intake motors to be accessed and
 *                  set in one line.
 *
 * Note:            None
 ******************************************************************************/
void setIntakeMotors(int leftMotor, int rightMotor)
{
    _LATB5 = leftMotor;
    _LATA4 = rightMotor;
}

/*******************************************************************************
 * Function:        void forwardLineFollow(int speed)
 *
 * PreCondition:    None
 *
 * Input:           int speed (0-1023)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function consists of the basic line following
 *                  algorithm. If the front left sensor detects the line, the
 *                  robot turns left. If the front right sensor detects the
 *                  line, the robot turns right. In any other case, the robot
 *                  drives forward.
 *
 * Note:            Because the Challenge functions use a while loop, this
 *                  function doesnt require a loop to execute properly.
 ******************************************************************************/
void forwardLineFollow(int speed)
{
    LEFT_DRIVE_PWM = speed;
    RIGHT_DRIVE_PWM = speed;

    //Front Left Sensor detects line
    if (frontLeftOpto < frontLeftThreshold)
    {
        turnLeft(speed);    //Robot turns left
    }

    //Front Right Sensor detects line
    else if (frontRightOpto < frontRightThreshold)
    {
        turnRight(speed);   //Robot turns right
    }

    else //Both front sensors detect white or black
    {
        setDriveMotors(0,1,1,0);    //Robot drives forward
    }
}

/*******************************************************************************
 * Function:        void backwardLineFollow(int speed)
 *
 * PreCondition:    None
 *
 * Input:           int speed (0-1023)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function consists of the basic line following
 *                  algorithm, modified to control the robot driving backwards.
 *                  If the back left sensor detects the line, the robot turns
 *                  right. If the back right sensor detects the line, the robot
 *                  turns left. In any other case, the robot drives backward.
 *
 * Note:            Because the Challenge functions use a while loop, this
 *                  function doesnt require a loop to execute properly.
 ******************************************************************************/
void backwardLineFollow(int speed)
{
    RIGHT_DRIVE_PWM = speed;
    LEFT_DRIVE_PWM = speed;

    //Back Left Sensor detects line
    if (backLeftOpto < backLeftThreshold)
    {
        turnRight(speed);       //Robot turns left
    }

    //Back Left Sensor detects line
    else if (backRightOpto < backRightThreshold)
    {
        turnLeft(speed);        //Robot turns right
    }

    else    //Both back sensors detect white or black
    {
        setDriveMotors(1,0,0,1);    //Robot drives forward
    }
}

/*******************************************************************************
 * Function:        void driveForward(int speed, int time)
 *
 * PreCondition:    None
 *
 * Input:           int speed (0-1023)
 *                  int time (milliseconds)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function drives the robot forwards at a desired speed
 *                  for a given time. It is used to align the robot in the home
 *                  square.
 *
 * Note:            None
 ******************************************************************************/
void driveForward (int speed, int time)
{
    LEFT_DRIVE_PWM = speed;         //Robot drives forward
    RIGHT_DRIVE_PWM = speed;
    setDriveMotors(0,1,1,0);

    delayMs(time);

    LEFT_DRIVE_PWM = 0;             //Robot stops
    RIGHT_DRIVE_PWM = 0;
    setDriveMotors(0,0,0,0);
}

/*******************************************************************************
 * Function:        void driveBackward(int speed, int time)
 *
 * PreCondition:    None
 *
 * Input:           int speed (0-1023)
 *                  int time (milliseconds)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function drives the robot backwards at a desired speed
 *                  for a given time. It is used to align the robot in the home
 *                  square. The right side is driven slower because, through
 *                  testing, the robot was turning to the left.
 *
 * Note:            None
 ******************************************************************************/
void driveBackward (int speed, int time)
{
    LEFT_DRIVE_PWM = speed;         //Robot drives backward
    RIGHT_DRIVE_PWM = speed - 100;
    setDriveMotors(1,0,0,1);

    delayMs(time);

    LEFT_DRIVE_PWM = 0;             //Robot stops
    RIGHT_DRIVE_PWM = 0;
    setDriveMotors(0,0,0,0);
}

/*******************************************************************************
 * Function:        void turnLeft(int speed)
 *
 * PreCondition:    None
 *
 * Input:           int speed (0-1023)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function sets the drive PWM values to the desired speed
 *                  and runs the left motor in reverse and the right motor
 *                  forwards. This makes the robot turn left.
 *
 * Note:            Because the Challenge functions use a while loop, this
 *                  function doesnt require a loop to execute properly.
 ******************************************************************************/
void turnLeft(int speed)
{
    LEFT_DRIVE_PWM = speed;
    RIGHT_DRIVE_PWM = speed;
    setDriveMotors(1,0,1,0);
}

/*******************************************************************************
 * Function:        void turnRight(int speed)
 *
 * PreCondition:    None
 *
 * Input:           int speed (0-1023)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function sets the drive PWM values to the desired speed
 *                  and runs the right motor in reverse and the left motor
 *                  forwards. This makes the robot turn right.
 *
 * Note:            Because the Challenge functions use a while loop, this
 *                  function doesnt require a loop to execute properly.
 ******************************************************************************/
void turnRight(int speed)
{
    LEFT_DRIVE_PWM = speed;
    RIGHT_DRIVE_PWM = speed;
    setDriveMotors(0,1,0,1);
}

/*******************************************************************************
 * Function:        void turnLeftUntilLine(int speed)
 *
 * PreCondition:    None
 *
 * Input:           int speed (0-1023)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function makes the robot turn left until the front left
 *                  sensor detects the line.
 *
 * Note:            Because the Challenge functions use a while loop, this
 *                  function doesnt require a loop to execute properly.
 ******************************************************************************/
void turnLeftUntilLine(int speed)
{
    if(frontLeftOpto > frontLeftThreshold)
    {
        turnLeft(speed);
    }

    else if (frontLeftOpto < frontLeftThreshold)
    {
        /*Continues to turn the robot for 100ms more in order to center the
         *robot on the line*/
        delayMs(100);
        LEFT_DRIVE_PWM = 0;         //Stop robot
        RIGHT_DRIVE_PWM = 0;
        setDriveMotors(0,0,0,0);
        robotState++;
    }
}

/*******************************************************************************
 * Function:        void turnRightUntilLine(int speed)
 *
 * PreCondition:    None
 *
 * Input:           int speed (0-1023)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function makes the robot turn right until the front
 *                  right sensor detects the line.
 *
 * Note:            Because the Challenge functions use a while loop, this
 *                  function doesnt require a loop to execute properly.
 ******************************************************************************/
void turnRightUntilLine(int speed)
{
    if(frontRightOpto > frontRightThreshold)
    {
        turnRight(speed);
    }

    else if (frontRightOpto < frontRightThreshold)
    {
        /*Continues to turn the robot for 100ms more in order to center the
         *robot on the line*/
        delayMs(100);
        LEFT_DRIVE_PWM = 0;         //Stop robot
        RIGHT_DRIVE_PWM = 0;
        setDriveMotors(0,0,0,0);
        robotState++;
   }
}

/*******************************************************************************
 * Function:        void clearLineForward(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to move the robot forwards after the
 *                  back sensors detect the junction. Its a specific case of
 *                  the driveForward function.
 *
 * Note:            None
 ******************************************************************************/
void clearLineForward(void)
{
    LEFT_DRIVE_PWM = 1023;      //Set robot to full speed forward
    RIGHT_DRIVE_PWM = 1023;
    setDriveMotors(0,1,1,0);

    delayMs(400);               //For 400ms

    LEFT_DRIVE_PWM = 0;         //Stop robot
    RIGHT_DRIVE_PWM = 0;
    setDriveMotors(0,0,0,0);
}

/*******************************************************************************
 * Function:        void clearLineBackward(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to move the robot backwards after the
 *                  front sensors detect the junction. Its a specific case of
 *                  the driveBackward function.
 *
 * Note:            None
 ******************************************************************************/
void clearLineBackward(void)
{
    LEFT_DRIVE_PWM = 1023;      //Set robot to full speed backward
    RIGHT_DRIVE_PWM = 1023;
    setDriveMotors(1,0,0,1);

    delayMs(500);               //For 400ms

    LEFT_DRIVE_PWM = 0;         //Stop robot
    RIGHT_DRIVE_PWM = 0;
    setDriveMotors(0,0,0,0);
}

/*******************************************************************************
 * Function:        void frontCounter(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function checks whether or not both front sensors have
 *                  detected the line (junction). If so, the robot moves back
 *                  a little to clear the line and the counters are updated.
 *                  It allows the robot to transition from forwards line
 *                  following to backwards line following.
 *
 * Note:            None
 ******************************************************************************/
void frontCounter(void)
{
    if ((frontLeftOpto < frontLeftThreshold) &&
            (frontRightOpto < frontRightThreshold))
    {
        lineCounter += 1;
        clearLineBackward();
        robotState += 1;
    }
}

/*******************************************************************************
 * Function:        void backCounter(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function checks whether or not both back sensors have
 *                  detected the line (junction). If so, the robot moves forward
 *                  a little to clear the line and the counters are updated.
 *                  It allows the robot to transition from backwards line
 *                  following to forwards line following.
 *
 * Note:            None
 ******************************************************************************/
void backCounter(void)
{
    if ((backLeftOpto < backLeftThreshold) &&
            (backRightOpto < backRightThreshold))
    {
        lineCounter += 1;
        clearLineForward();
        robotState += 1;
    }
}

/*******************************************************************************
 * Function:        void driveWithIntake(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function allows the robot to intake the pail. It makes
 *                  use of an intake button and some timeout logic to run the
 *                  intake. The intake runs until the button is pressed (from
 *                  the pail hitting it) or 200 cycles of the loop have
 *                  executed. The robot drives forward while intaking to allow
 *                  the pail to be sucked in more easily.
 *
 * Note:            None
 ******************************************************************************/
void driveWithIntake()
{
    button = _RB9;      //Read state of intake button

   //While intake button is not pressed
    while(button == 1)
    {
        loopCounter++;              //Increment cycle counter
        if (loopCounter > 200)      //If 200 cycles have passed, stop intake
        {
           break;
        }
        button = _RB9;

        LEFT_DRIVE_PWM = 400;       //Drive forwards slowly
        RIGHT_DRIVE_PWM = 400;
        setDriveMotors (0,1,1,0);

        setIntakeMotors (1,0);      //Intake
        delayMs(10);                //Keeps loop running at a reasonable rate
                                    //so loopCounter won't be very large.
    }

    loopCounter = 0;                //Reset cycle counter
    setIntakeMotors(0,0);           //Turn off intake motors

    LEFT_DRIVE_PWM = 0;             //Stop robot
    RIGHT_DRIVE_PWM = 0;
    setDriveMotors(0,0,0,0);
    
    robotState++;
}

/*******************************************************************************
 * Function:        void outtake(int time)
 *
 * PreCondition:    None
 *
 * Input:           int time (milliseconds)
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function allows the robot to outtake for a desired
 *                  amount of time. It is used to drop off the pail.
 *
 * Note:            None
 ******************************************************************************/
void outtake (int time)
{
    setIntakeMotors(0,1);
    delayMs(time);
    setIntakeMotors(0,0);
    robotState++;
}

/*******************************************************************************
 * Function:        void senseTemperature(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function samples the thermistor and reads the value
 *                  that it returns. It is used to detect the temperature of the
 *                  pail in Challenge Three. This function also defines the path
 *                  that the robot must take to drop off the pail (for Challenge
 *                  Three) based on the value of the thermistor. 
 *
 * Note:            None
 ******************************************************************************/

void senseTemperature(void)
{
    delayMs(1500);          //Allow thermistor time to adapt to new temperature
    AD1CHS = 0x0000;        //Connect RA1/AN1 as CH0 input
    _SAMP = 1;              //Sample potentiometer value
    delayMs(5);             //after 5mS start conversion
    _SAMP = 0;              //Convert thermistor value
    while(!_DONE);          //conversion done? (takes 12*Tad)
    thermistor = ADC1BUF0;  //yes, then save ADC value

   //Pail is "Too Hot"
    if (thermistor < 710) 
    {
        path = RIGHT;
    }

   //Pail is "Just Right" (Room Temperature)
    else if (thermistor >= 710 && thermistor < 720) 
    {
        path = STRAIGHT;
    }

   //Pail is "Too Cold"
    else if (thermistor >= 720) 
    {
        path = LEFT;
    }
}

/*******************************************************************************
 * Function:        void delayMs(unsigned int ms)
 *
 * PreCondition:    Requires Tcyc=250nS (8 MHz Fosc)
 *
 * Input:           delay in milliseconds (1-65535)
 *
 * Output:          None
 *
 * Side Effects:    Blocking Delay (CPU is blocked from doing other tasks)
 *                  Non-portable (Uses PIC24 Assembly language instructions)
 *
 * Overview:        This function implements an in-line delay of up to 65535ms
 *
 * Note:            None
 ******************************************************************************/
void delayMs(unsigned int ms)
{
   while(ms--)
   {
       asm("repeat #4000"); // 4000 instruction cycles @ 250nS ea. = 1mS
       asm("nop");          // instruction to be repeated 4000x
   }
}
//End mainLineFollower.c





