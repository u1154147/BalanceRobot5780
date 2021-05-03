# PS3 Controller Communication Module
Program to handle BlueTooth connection with a PS3 controller. The controller connect to a USB adapter, where the adapter is plugged into a [USB Host Shield](https://www.sparkfun.com/products/9947), which communicates to the Discovery board via SPI. Currently, sends data received to a USART connection. This program uses the pins listed below: 
  - SPI: PC2, PC3, PB10: SCK, MISO, and MOSI, respectively.
  - USART: PC10 and PC11 - TX and RX, respectively.
