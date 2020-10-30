# STM32F103_UART_IT
This application was developed as a work in the discipline of Embedded Systems Programming at UFMG - Prof. Ricardo de Oliveira Duarte - Department of Electronic Engineering.

It is an example of how to use UART in interrupt mode with STM32F1 family microcontroller, and was tested with STM32F103C8T6 microcontroller (blue pill).

### Hardware:
- USB UART converter (TTL)
- 2 LEDs
- Display OLED (SSD1306)

### Pinout:
- A1  - LED_RX
- A2  - LED_TX
- A9  - TTL RX pin
- A10 - TTL TX pin
- B6  - OLED SCK pin
- B7  - OLED SDA pin

### Description:

When the microcontroller receives a character from the computer, the OLED displays the string formed by the characters received until that moment.
If the character received is '\r' (end of the message), the OLED displays the final string and the MCU sends it to the computer.
LED_RX blinks when a character is received, and LED_TX blinks when a message is sent. 

The softwares used in the project were STM32CubeMX and System Workbench for STM32 (SW4STM32).
The configuration of the peripherals used are:
- USART1: Asynchronous, Baud Rate = 9600 bits/s
- I2C1: Fast Mode with 400kHz clock speed

(See the .ioc file for more details)

The software utilized to send data via UART to the microcontroller was Realterm. 
To use it, in the 'Port' tab, select a Baud rate of 9600 bps and the COM port where the TTL converter is connected, then click 'Change'.
In this application, the STM32 receives only one character at a time.
To send a character in Realterm, just click in the screen and type the desired character. 
To send '\r', click ENTER.

The library used to communicate with the OLED display can be downloaded in:
https://github.com/SL-RU/stm32libs

### Contact:
In case of doubts or problems, please contact the developers:
- Carolina Mayumi M. Guimarães - carolmmg@hotmail.com
- Eduardo Tschaen Filho - tschaenfilho@gmail.com
