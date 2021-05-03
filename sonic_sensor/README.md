# Sonic Sensor
Programmed pins PB9 and PB8 for the sonic sensor. Sonic sensor sends 10us trigger pulse on trigger pin (PB8) and then sends sonic pulses to use for echoing off of distant 
surfaces. PB9 reads echo pin on sensor and calculates distance using the speed of sound in centimeters per microsecond and time that the echo pin was digital high. 

## Results 
Currently does not work unfortunately. Our use of timers was not able to properly keep time and the trigger pin was unable to stay high for only 10 microseconds thus no sonic 
pulse was output.
