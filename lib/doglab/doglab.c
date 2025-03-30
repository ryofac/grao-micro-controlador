#include <stdlib.h>
#include <stdio.h>

#include "doglab.h"
#include "ssd1306.h"
#include "hardware/adc.h"

// Aloca a variável do display da tela para ser manipulada pelas funções de display
ssd1306_t display;

void init_i2c()
{
  // Inicializa I2C no canal 1
  i2c_init(i2c1, 400 * 1000);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);
}

void init_display()
{
  // Inicializa o display
  if (!ssd1306_init(&display, SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_ADDRESS, i2c1))
  {
    printf("Display não iniciado");
  }
  else
  {
    printf("Display SSD1306 iniciado com sucesso!");
  }
}

void init_leds()
{
  int leds[3] = {LED_RED, LED_GREEN, LED_BLUE};
  for (int i = 0; i < 3; i++)
  {
    gpio_init(leds[i]);
    gpio_set_dir(leds[i], GPIO_OUT);
    gpio_put(leds[i], 0);
  }
}

void init_buzzer()
{
  gpio_init(BUZZER_PIN_1);
  gpio_set_dir(BUZZER_PIN_1, GPIO_OUT);
}

void init_analog()
{
  adc_init();
  adc_gpio_init(ANALOG_X);
  adc_gpio_init(ANALOG_Y);
  gpio_init(ANALOG_BTN);
  gpio_set_dir(ANALOG_BTN, GPIO_IN);
  gpio_pull_up(ANALOG_BTN);
}

void init_adc()
{
  adc_init();
  adc_set_temp_sensor_enabled(true);
}

void init_all()
{
  stdio_init_all();
  init_i2c();
  init_display();
  init_leds();
  init_adc();
}
