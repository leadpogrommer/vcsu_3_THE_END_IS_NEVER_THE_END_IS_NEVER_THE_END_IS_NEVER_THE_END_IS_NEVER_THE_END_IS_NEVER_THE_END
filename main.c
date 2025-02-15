#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/spi.h"
#include "libopencm3/stm32/dma.h"

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
    SPI1_DR = data;
    while (!(SPI1_SR & SPI_SR_TXE)); // wait for transfer to end
}

void matrix_send(uint16_t addr, uint16_t data){
    matrix_send_raw(((addr & 0x0f) << 8) | (data & 0xff));

}

void matrix_draw_image(uint8_t* data){
    for(int i = 0; i < 8; i++){
        matrix_send(i+1, data[i]);
    }
}

void matrix_send_dma(uint16_t *data, int len){
    DMA2_S2CR &= ~DMA_SxCR_EN; // stop stream

    DMA2_S2PAR = &SPI1_DR;
    DMA2_S2M0AR = data;
    DMA2_S2NDTR = len;


    DMA2_S2CR = DMA_SxCR_DIR_MEM_TO_PERIPHERAL | DMA_SxCR_PL_LOW | DMA_SxCR_MSIZE_16BIT | DMA_SxCR_PSIZE_16BIT | DMA_SxCR_MINC;
    DMA2_S2CR = DMA_SxCR_CHSEL(2);

    DMA2_S2CR |= DMA_SxCR_EN;
}

void matrix_init(){
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable gpio a
    RCC_APB2ENR |= RCC_APB2ENR_SPI1EN; // enable spi 1
    RCC_AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    GPIOA_MODER |= (0b10 << 4*2) | (0b10 << 5*2) | (0b10 << 7*2);
    GPIOA_AFRL  |= (5 << 4*4)    | (5 << 5*4)    | (5 << 7*4)   ;


    SPI1_CR1 |= SPI_CR1_BIDIMODE_2LINE_UNIDIR | SPI_CR1_BIDIOE | SPI_CR1_DFF_16BIT | SPI_CR1_MSTR | SPI_CR1_BAUDRATE_FPCLK_DIV_2; // WE GO SLOW
    SPI1_CR2 |= SPI_CR2_SSOE | SPI_CR2_FRF_TI_MODE | SPI_CR2_TXDMAEN;
    SPI1_CR1 |= SPI_CR1_SPE;

    // IDK why, but sometimes it doesn't work
    uint16_t data[] = {0xaba, 0b10100001};
    matrix_send_dma(data, 2);

//    matrix_send(0x09, 0); // no decode
//    matrix_send(0x0C, 1); // no shutdown
//    matrix_send(0x0b, 0b111); // no scan-limit
//    matrix_send(0x0a, 1); // intensity
}

int main(){
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC_MODER |= GPIO_MODE_OUTPUT << (13*2);

    matrix_init();
//    matrix_draw_image(nsu);


    while (1){
        GPIOC_ODR |= 1 << 13;
        delay(DELAY);
        GPIOC_ODR &= ~(1 << 13);
        delay(DELAY);
//        matrix_send_raw(0b1010000000000001);
    }
}