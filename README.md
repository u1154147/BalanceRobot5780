# RCTable

## PHOTOS:
![pic1](https://github.com/u1154147/RCTable/blob/main/photos/img1.png)
![pic5](https://github.com/u1154147/RCTable/blob/main/photos/img5.png)
![pic9](https://github.com/u1154147/RCTable/blob/main/photos/img9.png)

## PURPOSE: 
A robot that can self-balance and can be remote controlled and even has some ability to detect obstacles and stop. This is a proof of concept because it requires a lot of motor control which has many applications in robotics. Motor control provides precision and fine movement in applications like robotic surgery. 

## SETUP: 
To setup a simple balancing robot, one will need:
- 1 microcontroller with a gyroscope or IMU like an STM32 discovery board, Cortex M0 (and USB upload cable) 
- 1 or 2 breadboards, 1 or 2 motor drivers (using L298N H-Bridges) 
- 2 DC motors with encoders 
- 2 large rubber wheels 
- 2 motor mounts 
- 1 or 2 6V battery pack 
- 1 PS3 or PS4 controller 
- 1 USB host shield 
- 1 or 2 sonic sensors 
- Various resistors for voltage dividers
- Various jumper wires  
- A wooden (may need wood glue to attach pieces) or PCB proto board frame to house the microcontroller, motor drivers, etc. 

One will then need to develop code similar to ours to control motors using PID control, code to read gyroscope or IMU axis changes, code to read distance with a sonic sensor, and code to setup and control motors using a bluetooth controller.

Then, one needs to wire the proper chosen microcontroller pins to the motor drivers with the motor wires attached to the drivers. Power must be wired from the battery pack to the microcontroller and motor drivers by voltage dividers with chosen resistors. Motors must be mounted and attached to the bottom of the wooden or proto board frame. The USB host shield must be attached anywhere on the frame but near to the microcontroller and its wires. Sonic sensors must be mounted on the front or back or both on the frame in a steady spot to detect obstacles. Use double sided foam tape to mount breadboards, motor drivers, etc. Use jumper cables and breadboards to properly connect all microcontroller pins to various inputs and outputs. Upload code to microcontroller and enjoy!

## CODE LAYOUT:

If you want to generate your own project using STM32CubeMX or another piece of software used to upload code to the microcontroller, then simply place the Src and Inc code files into the 'Source' and 'Include' folders appropriate to your program's file directory. For example, wherever your ```main.c``` file is, you can place all the 'Src' files into here for one of the projects. 

If you simply want to plug-and-play, install MDK-ARM V5 and unzip one of the [FULL_PROJECTS](https://github.com/u1154147/RCTable/tree/main/FULL_PROJECTS) zip files to have a fully operational project. 

## MILESTONES:
- [x] Discovery board can accurately control motors. [(Relevant files)](motor_control)
- [x] Accurate gyroscope readings and motor feedback. [(Relevant files)](motor_control)
- [ ] Bluetooth connected and can exert control over direction and driving. [(Relevant files)](ps3_controller_comm)
- [ ] Sonar for obstacle detection, robot can stop when obstacle detected and balance itself. [(Relevant files)](sonic_sensor)

To see more details on implementation details for completed milestones, or issues encountered in uncompleted milestones, see the appropriate section listed above.

## PROJECT NAME ORIGIN: 
It is like a table and was going to be remote controlled. Ergo, RC (remote-controlled) Table.


## AUTHORS:
- Greyson Mitra
- Bhavani SampathKumar
- Zach Phelan
