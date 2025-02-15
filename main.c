#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/spi.h"

#define DELAY 300000
void delay(int d){
    volatile int c = 0;
    for (int i = 0; i < d; i++){
        c = i;
    }
}

uint8_t nsu[] = {
    0b01000000,
    0b11100000,
    0b01000000,
    0b00011110,
    0b00000100,
    0b00001000,
    0b00011110,
    0b00000000
};



void matrix_send_raw(uint16_t data){
    GPIOA_BSRR |= (1 << (16+4));
    SPI1_DR = data;
    while (!(SPI1_SR & SPI_SR_TXE) || (SPI1_SR & SPI_SR_BSY)); // wait for transfer to end
//    delay(50); // WTF
    GPIOA_BSRR |= (1 << 4);

    delay(100);// TODO: remove this
}

void matrix_send(uint16_t addr, uint16_t data){
    matrix_send_raw(((addr & 0x0f) << 8) | (data & 0xff));

}

void matrix_draw_image(uint8_t* data){
    for(int i = 0; i < 8; i++){
        matrix_send(i+1, data[i]);

    }
}

void matrix_init(){
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable gpio a
    RCC_APB2ENR |= RCC_APB2ENR_SPI1EN; // enable spi 1
//    GPIOA_MODER |= (0b10 << 4*2) | (0b10 << 5*2) | (0b10 << 7*2);
//    GPIOA_AFRL  |= (5 << 4*4)    | (5 << 5*4)    | (5 << 7*4)   ;
    GPIOA_MODER |= (0b01 << 4*2) | (0b10 << 5*2) | (0b10 << 7*2);
    GPIOA_AFRL  |=                 (5 << 5*4)    | (5 << 7*4)   ;

    GPIOA_BSRR |= (1 << 4);


    SPI1_CR1 |= SPI_CR1_BIDIMODE_2LINE_UNIDIR | SPI_CR1_BIDIOE | SPI_CR1_DFF_16BIT | SPI_CR1_MSTR | SPI_CR1_BAUDRATE_FPCLK_DIV_256 | SPI_CR1_SSI | SPI_CR1_SSM; // WE GO SLOW
//    SPI1_CR2 |= SPI_CR2_SSOE | SPI_CR2_FRF_TI_MODE;
    SPI1_CR1 |= SPI_CR1_SPE;

    matrix_send(0x09, 0); // no decode
    matrix_send(0x0C, 1); // no shutdown
    matrix_send(0x0b, 0b111); // no scan-limit
    matrix_send(0x0a, 1); // intensity
}

int main(){
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC_MODER |= GPIO_MODE_OUTPUT << (13*2);

    delay(10000);
    matrix_init();


    while (1){
//        GPIOC_ODR |= 1 << 13;
//        delay(DELAY);
//        GPIOC_ODR &= ~(1 << 13);
//        delay(DELAY);
        matrix_draw_image(nsu);
//        matrix_send_raw(0b1010000000000001);
    }
}