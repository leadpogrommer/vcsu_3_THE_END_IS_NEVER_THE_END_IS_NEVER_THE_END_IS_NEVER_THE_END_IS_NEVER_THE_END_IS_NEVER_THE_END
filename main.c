#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"

#define DELAY 300000
void delay(int d){
    volatile int c = 0;
    for (int i = 0; i < d; i++){
        c = i;
    }
}

int main(){
    RCC_AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC_MODER |= GPIO_MODE_OUTPUT << (13*2);

    while (1){
        GPIOC_ODR |= 1 << 13;
        delay(DELAY);
        GPIOC_ODR &= ~(1 << 13);
        delay(DELAY);
    }
}