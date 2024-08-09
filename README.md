# STM32F103 Button Panel for LinuxCNC
Uses 18 hardware buttons, 6 LEDs and 4 Potentiometers (test with both 10k and 100k and works fine)

Tested with an STM32F103 BluePill with Resistor Fix:
 - R10 should be a R1.5k resistor (CODE: 152) not a R10k (CODE: 103)
 - see: https://amitesh-singh.github.io/stm32/2017/05/27/Overcoming-wrong-pullup-in-blue-pill.html
