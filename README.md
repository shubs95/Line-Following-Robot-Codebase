Line Following Robot Codebase
=============================

Software for a line-following robot constructed as a final project in Grade 12 Computer Engineering. Written in embedded C for a PIC24F microcontroller.

### Hardware Used:
PIC24FV32KA302 MCU  
2 Solarbotics GM9 DC Motors (Drivetrain)    
2 VEX Robotics 269 DC Motors (Intake system)    

4 Line Following Sensors (infrared)     
4 LED indicators for line following sensors  

Potentiometer  
Pushbuttons  
Thermistor    

Drive System H-Bridge (SN754410NE IC)  
Intake System H-Bridge (SN754410NE IC) 

### Overview:
This program is the codebase for a line following robot that was created to
participate in the Robots Get Grimm CETA Robotics Competition. The main function of the robot 
and program was to follow a black line on a white board as well as carry an aluminum pail.
The robot that was created had four line following sensors; two in the front of
the robot and two in the back. This was to allow the robot to follow the line
both forwards and backwards. The mechanism to carry the pail was an intake
system that consisted of two rollers, each individually powered. The intake
"sucked in" the pail onto to a plate. There was a thermistor used to detect
the temperature of the pail for Challenge Three and a pushbutton to detect the
the pail in the intake system.
