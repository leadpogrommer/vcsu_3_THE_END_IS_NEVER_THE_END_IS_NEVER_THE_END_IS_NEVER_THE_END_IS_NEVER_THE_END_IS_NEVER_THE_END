#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"

#include "main.h"


void init_clock(){
    // Единственное использование библиотечной функции
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_96MHZ]);

    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable gpio a
    RCC_APB2ENR |= RCC_APB2ENR_SPI1EN; // enable spi 1
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;
}

void init_gpio(){
    GPIOC_MODER |= GPIO_MODE_OUTPUT << (13 * 2); // LED output

    // matrix spi
    GPIOA_MODER |= (0b01 << 4 * 2) | (0b10 << 5 * 2) | (0b10 << 7 * 2);
    GPIOA_AFRL |=     (5 << 5 * 4) | (5 << 7 * 4);

    // uart
    GPIOA_MODER |= (0b10 << 9 * 2); // PA9 - Alternative function - tx
    GPIOA_AFRH |= (7 << 1 * 4);
    GPIOA_OTYPER |= GPIO_OTYPE_OD << (9);
}

__attribute__((noreturn)) int main() {
    init_clock();
    init_gpio();
    matrix_init();
    init_thermometer();

    while (1);
}