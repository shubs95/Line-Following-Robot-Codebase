Line-Following-Robot-Codebase
=============================

Software for a line-following robot constructed as a final project in Grade 12 Computer Engineering. Written in embedded C for a PIC24F microcontroller.

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
Front Left Line Following Sensor RB4/AN15
Front Right Line Following Sensor RB2/AN4
Back Left Line Following Sensor RA2/AN13
Back Right Line Following Sensor RA1/AN1
Front Left LED Indicator RB6
Front Right LED Indicator RB1/AN3
Back Left LED Indicator RB8
Back Right LED Indicator RB0/AN2
Potentiometer RB3/AN5
Intake Button RB9
Thermistor RA0/AN0
Drive System H-Bridge (SN754410NE IC)
1-2EN RB11/OC2
3-4EN RB10/OC3
1A (Left Drive Motor) RB13/AN11
2A (Left Drive Motor) RB12/AN12
3A (Right Drive Motor) RB15/AN9
4A (Right Drive Motor) RB14/AN10
Intake System H-Bridge (SN754410NE IC)
1-2EN +6V
3-4EN +6V
1A RB5
2A RA4
3A Wired to 2A
4A Wired to 1A
