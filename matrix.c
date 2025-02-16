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
};

uint16_t matrix_data_list[] = {
        0x0100,
        0x0200,
        0x0300,
        0x0400,
        0x050a,
        0x060b,
        0x0700,
        0x0800,
};

uint8_t font[][3] = {
        {
                0b00011111,
                0b00010001,
                0b00011111,
        },
        {
                0b00011111,
                0b00000010,
                0b00000000,
        },
        {

                0b00010111,
                0b00010101,
                0b00011101,
        },
        {
                0b00011111,
                0b00010101,
                0b00010001,
        },
        {
                0b00011111,
                0b00000100,
                0b00000111,
        },
        {
                0b00011101,
                0b00010101,
                0b00010111,

        },
        {
                0b00011111,
                0b00010100,
                0b00011100,
        },
        {
                0b00011111,
                0b00000001,
                0b00000001,
        },
        {
                0b00011111,
                0b00010101,
                0b00011111,
        },
        {
                0b00011111,
                0b00000101,
                0b00000111,
        },
};


volatile int matrix_send_step = 0;
volatile uint16_t *volatile matrix_send_buffer;
volatile int matrix_send_len;

void spi1_isr() {
    if (SPI1_SR & SPI_SR_TXE) {
        if (matrix_send_step < matrix_send_len) {
            SPI1_DR = matrix_send_buffer[matrix_send_step++];
        } else {
            NVIC_ICER(NVIC_SPI1_IRQ / 32) = 1 << (NVIC_SPI1_IRQ % 32);
        }
    }
}

void matrix_send_buffer_async(uint16_t *data, int len) {
    matrix_send_step = 0;
    matrix_send_buffer = data;
    matrix_send_len = len;
    NVIC_ISER(NVIC_SPI1_IRQ / 32) = 1 << (NVIC_SPI1_IRQ % 32);
}

void matrix_set_digit(int d, int place) {
    for (int i = 0; i < 3; i++) {
        int y = place * 4 + i;
        matrix_data_list[y] = ((y + 1) << 8) | font[d][2 - i];
    }
}

void print_num(int n) {
    matrix_set_digit(n / 10, 0);
    matrix_set_digit(n % 10, 1);
    matrix_send_buffer_async(matrix_data_list, 8);
}

void matrix_init() {
    SPI1_CR1 |= SPI_CR1_BIDIMODE_2LINE_UNIDIR | SPI_CR1_BIDIOE | SPI_CR1_DFF_16BIT | SPI_CR1_MSTR |
                SPI_CR1_BAUDRATE_FPCLK_DIV_256; // WE GO SLOW
    SPI1_CR2 |= SPI_CR2_SSOE | SPI_CR2_FRF_TI_MODE | SPI_CR2_TXEIE;
    SPI1_CR1 |= SPI_CR1_SPE;

    matrix_send_buffer_async(matrix_init_list, 4);
    // Блокируемся до конца инициализации
    while (matrix_send_step != 4);
}
