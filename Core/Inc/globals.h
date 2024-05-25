#ifndef __GLOBALS_H
#define __GLOBALS_H

#ifdef __cplusplus
extern "C" {
#endif


#define BTN_READ_FREQUENCY 10
#define ADC_READ_FREQUENCY 10
#define ADC_SERIAL_SEND_FREQUENCY 1000

#define NUM_BUTTONS 19
#define NUM_LEDS 6
#define NUM_DIALS 4

#define LED_X GPIOB, GPIO_PIN_0
#define LED_Y GPIOA, GPIO_PIN_7
#define LED_Z GPIOA, GPIO_PIN_6

#define LED_1 GPIOB, GPIO_PIN_11
#define LED_2 GPIOB, GPIO_PIN_10
#define LED_3 GPIOB, GPIO_PIN_1

#define BTN_SPINDLE GPIOC, GPIO_PIN_14
#define BTN_BONUS GPIOC, GPIO_PIN_15

#define BTN_V_0 GPIOB, GPIO_PIN_4
#define BTN_V_1 GPIOB, GPIO_PIN_5
#define BTN_V_2 GPIOB, GPIO_PIN_6
#define BTN_V_3 GPIOB, GPIO_PIN_7
#define BTN_V_4 GPIOB, GPIO_PIN_8
#define BTN_V_5 GPIOB, GPIO_PIN_9
#define BTN_V_6 GPIOC, GPIO_PIN_13

#define BTN_H_0 GPIOB, GPIO_PIN_3
#define BTN_H_1 GPIOA, GPIO_PIN_15
#define BTN_H_2 GPIOA, GPIO_PIN_10
#define BTN_H_3 GPIOA, GPIO_PIN_9
#define BTN_H_4 GPIOA, GPIO_PIN_8
#define BTN_H_5 GPIOB, GPIO_PIN_15
#define BTN_H_6 GPIOB, GPIO_PIN_14
#define BTN_H_7 GPIOB, GPIO_PIN_13
#define BTN_H_8 GPIOB, GPIO_PIN_12
#define BTN_H_9 GPIOA, GPIO_PIN_2

extern uint8_t leds[NUM_LEDS];


#ifdef __cplusplus
}
#endif

#endif /* __GLOBALS_H */
