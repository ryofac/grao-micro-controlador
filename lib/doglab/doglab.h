#ifndef DOGLAB_H
#define DOGLAB_H

// Botões:
#define BTA 5
#define BTB 6

#define LED_RED 12
#define LED_GREEN 13
#define LED_BLUE 11

#define ANALOG_X 27
#define ANALOG_Y 26
#define ANALOG_BTN 22
#define BUZZER_PIN_1 21
#define BUZZER_PIN_2 10

// Variáveis Display OLED
#include "hardware/i2c.h"
#include "ssd1306.h"
extern ssd1306_t display;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define I2C_SDA 14 // Pino SDA
#define I2C_SCL 15 // Pino SCL

void init_i2c();

void init_display();

void init_leds();

void init_buzzer();

void init_analog();

void initButtons();

void init_all();

#endif