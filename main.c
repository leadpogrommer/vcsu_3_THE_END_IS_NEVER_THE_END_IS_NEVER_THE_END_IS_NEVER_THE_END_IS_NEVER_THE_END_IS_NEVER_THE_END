#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/usart.h"
#include "libopencm3/stm32/spi.h"
//#include "libopencm3/stm32/"
#include <libopencm3/cm3/nvic.h>

#include "main.h"

#define DELAY 800000

void delay(int d) {
    volatile int c = 0;
    for (int i = 0; i < d; i++) {
        c = i;
    }
}


void init_clock(){
    // Единственное использование библиотечной функции
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_96MHZ]);

    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable gpio a
    RCC_APB2ENR |= RCC_APB2ENR_SPI1EN; // enable spi 1
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
}

void init_gpio(){
    GPIOC_MODER |= GPIO_MODE_OUTPUT << (13 * 2); // LED output

    // matrix spi
    GPIOA_MODER |= (0b10 << 4 * 2) | (0b10 << 5 * 2) | (0b10 << 7 * 2);
    GPIOA_AFRL |= (5 << 4 * 4) | (5 << 5 * 4) | (5 << 7 * 4);

    // uart
    GPIOA_MODER |= (0b10 << 9 * 2) | (0b10 << 10 * 2);
    GPIOA_AFRH |= (7 << 1 * 4) | (7 << 2 * 4);
    GPIOA_OTYPER |= GPIO_OTYPE_OD << (9);
    GPIOA_PUPDR |= GPIO_PUPD_PULLUP << (10*2); // TODO: uncomment before connecting
}

void uart_set_baudrate(int rate){
    USART1_BRR = (96000000 + rate / 2) / rate;
}

uint8_t uart_do_one_wire_blocking(int baudrate, uint8_t data){
    uart_set_baudrate(baudrate);
    USART1_SR &= ~(USART_SR_RXNE);
    USART1_DR = data;
    while (!(USART1_SR & USART_SR_TC) || !(USART1_SR & USART_SR_RXNE));
    return USART1_DR;
}



uint8_t onewire_byte(uint8_t data){
    uint8_t res = 0;
    for(int i = 0; i < 8; i++){
        uint8_t txBit = (data >> i) & 0x01;
        uint8_t rxBit = 0;
        uint8_t  bit_res = uart_do_one_wire_blocking(115200, txBit ? 0xff: 0x00);
        if(bit_res == 0xff){
            res |= 1 << i;
        }
    }
    return res;
}


int init_uart(){
    USART1_CR1 |= USART_CR1_UE;
    uart_set_baudrate(115200);
    USART1_CR1 |= USART_CR1_TE | USART_CR1_RE;


//    while (!(USART1_SR & USART_SR_TC)); // wait
//    USART1_DR = 'h';
//    while (!(USART1_SR & USART_SR_TC)); // wait
//    USART1_DR = 'e';
//    while (!(USART1_SR & USART_SR_TC)); // wait
//    USART1_DR = 'l';
//    while (!(USART1_SR & USART_SR_TC)); // wait
//    USART1_DR = 'p';

    // 1-Wire test
    volatile uint8_t data = uart_do_one_wire_blocking(9600, 0xf0); // reset

    onewire_byte(0x33);

    uint8_t rom_contents[8];

    for(int i = 0; i < 8; i++){
        rom_contents[i] = onewire_byte(0xff);
    }

    return data;
}

int main() {
    init_clock();
    init_gpio();
//    matrix_init();
    init_uart();


    int i = 0;

    while (1) {
//        GPIOC_ODR |= 1 << 13;
//        delay(DELAY);
//        GPIOC_ODR &= ~(1 << 13);
//        delay(DELAY);
        print_num(i);
        i = (i + 1) % 100;
        delay(DELAY);
//        matrix_send_raw(0b1010000000000001);
    }
}