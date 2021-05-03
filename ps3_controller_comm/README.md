# PS3 Controller Communication Module
## Overview
Program to handle BlueTooth connection with a PS3 controller. The controller connect to a USB adapter, where the adapter is plugged into a [USB Host Shield](https://www.sparkfun.com/products/9947), which communicates to the Discovery board via SPI. Currently, sends data received to a USART connection. This program uses the pins listed below: 

## Pins Used
  - SPI 
    - PC2: MISO - Master *INPUT* Slave *OUTPUT*
    - PC3: MOSI - Master *OUTPUT* Slave *INPUT*
    - PA0: SCK - Clock for slave synchronization
  - USART 
    - PC1: TX - data to console.
    - PC11: RX - data from console.

## Functionality
We were not able to establish a clear connection to the USB adapter board. The USB Host Shield was designed for Arduino's in mind, so it was not intuitive for the Discover board. Additionally, we did not know how to begin pairing for the Bluetooth adapter and the PS3 controller. We were able to get the individual MAC addresses, but could not pair them. Even if the controller was plugged straight into the USB board, no data was transferred to the Discovery board.
