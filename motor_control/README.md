# Motor control code and logic

#  Overview
Motor control done using PID controller where the motor rpm is set based on gyroscope position and encoder measurements. The farther the gyroscope position is from the stable position the more the motor rpm increases. Gyroscope simply is configured and then reads changes in z-axis register. Borrows code from Labs 5 and 7 for gyroscope reading and motor control, respectively.  

# Pins 
 - Motor Wiring
    - PA4: PWM pin. Goes to 'Enable' pin on Motor Driver.
    - PA5 & PC4: Sets the direction of the motors. Goes to 'Input 1' and 'Input 2' pins on the Motor Driver, respectively,
    - PB4 & PB5: Read motor encoder to get speed and direction. 
    - 5V and GND: Digital Logic 
   
 - Used for I2C communication w/ Gyroscope. (Does not have to be wired, but pins are used.)
   - PB11
   - PB13
   - PB15

# Results

The motor driver is able to keep the structure standing for a few seconds, but then the robot falls. We believe that this is because the motor overshoots when it is returning back up, which causes gravity to be too much when it detects that it should change directions again. It is unfortunate, but we tried many different variables, techniques, and ideas to get the robot to stand for longer, but we cannot seem to get over this hurdle. 
