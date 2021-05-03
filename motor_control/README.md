Motor control code

Pins required:
  PA4 - H-Bridge PWM
  PA5 & PC4 - Motor Direction
  
  PB4 & PB5 - Motor Encoder pins
  
  GND and 5V pins for power
  
Motor control done using PID controller where the motor rpm is set based on gyroscope position and encoder measurements. The farther the gyroscope position is from the stable position the more the motor rpm increases. Borrows code from Labs 5 and 7 for gyroscope reading and motor control, respectively. 
  
   
GYROSCOPE I2C
  PB11, PB13, PB15   
