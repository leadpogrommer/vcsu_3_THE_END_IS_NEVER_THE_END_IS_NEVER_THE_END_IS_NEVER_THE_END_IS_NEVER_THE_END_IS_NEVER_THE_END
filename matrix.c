#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/spi.h"
#include <libopencm3/cm3/nvic.h>

#include "main.h"



uint16_t matrix_init_list[] = {
        0x0900,
        0x0c01,
        0x0b07,
        0x0a01,
        0x0000, // dummy byte to please the gods of crappy SPI
};

uint16_t matrix_data_list[] = {
        0x0100,
        0x0200,
        0x0300,
        0x0400,
        0x0500,
        0x0600,
        0x0700,
        0x0800,
        0x0000, // dummy byte to please the gods of crappy SPI
};



volatile int matrix_send_step = 0;
volatile uint16_t *volatile matrix_send_buffer;
volatile int matrix_send_len;

void spi1_isr() {
    if (SPI1_SR & SPI_SR_TXE) {
//        GPIOA_BSRR |= (1 << 4); // high
        GPIOA_BSRR |= (1 << (16+4)); // low

        if (matrix_send_step < matrix_send_len) {
            SPI1_DR = matrix_send_buffer[matrix_send_step++];
        } else {
            NVIC_ICER(NVIC_SPI1_IRQ / 32) = 1 << (NVIC_SPI1_IRQ % 32);
        }
    } else if (SPI1_SR & SPI_SR_RXNE) {
        GPIOA_BSRR |= (1 << 4); // high
        volatile int discard = SPI1_DR;
    }
}

void matrix_send_buffer_async(uint16_t *data, int len) {
    matrix_send_step = 0;
    matrix_send_buffer = data;
    matrix_send_len = len;
    NVIC_ISER(NVIC_SPI1_IRQ / 32) = 1 << (NVIC_SPI1_IRQ % 32); // interrupt will trigger since TXE is 1
}

void matrix_set_character(const uint8_t *data, int place){
    for (int i = 0; i < 3; i++) {
        int y = place * 4 + i+1;
        matrix_data_list[y] = ((y + 1) << 8) | data[2 - i];
    }
}

void matrix_set_digit(int d, int place) {
    matrix_set_character(font[d], place);
}

void matrix_flush(){
    matrix_send_buffer_async(matrix_data_list, 9);
}

void print_num(int n, int bar) {
    matrix_set_digit(n / 10, 0);
    matrix_set_digit(n % 10, 1);
    for(int i = 0; i < 8; i++){
        matrix_data_list[i] = (matrix_data_list[i] & 0b1111111101111111) | ((bar >= i) ? 0b10000000 : 0);
    }

    matrix_flush();
}

void matrix_init() {
    SPI1_CR1 |= SPI_CR1_BIDIMODE_2LINE_UNIDIR | SPI_CR1_BIDIOE | SPI_CR1_DFF_16BIT | SPI_CR1_MSTR |
                SPI_CR1_BAUDRATE_FPCLK_DIV_64; // WE GO SLOW
    SPI1_CR2 |= SPI_CR2_SSOE | SPI_CR2_TXEIE | SPI_CR2_RXNEIE;

//    SPI1_CR1 |= SPI_CR1_CPHA | SPI_CR1_CPOL;

    SPI1_CR1 |= SPI_CR1_SPE;

    matrix_send_buffer_async(matrix_init_list, 5);
    // Блокируемся до конца инициализации
    while (matrix_send_step != 5);
}
