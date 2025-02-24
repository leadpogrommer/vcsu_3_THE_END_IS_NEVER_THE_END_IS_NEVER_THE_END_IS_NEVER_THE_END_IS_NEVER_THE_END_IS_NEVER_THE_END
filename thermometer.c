#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/usart.h"
#include "libopencm3/stm32/spi.h"
#include "libopencm3/stm32/timer.h"
#include <libopencm3/cm3/nvic.h>
#include <memory.h>


#include "main.h"


typedef enum{
    TEMP_WAITING,
    TEMP_SENDING_CONFIG,
    TEMP_SENDING_CONVT,
    TEMP_READING_RAM
} temp_stage_t;

typedef enum {
    ERR_NONE,
    ERR_NO_DEVICE,
    ERR_TIMEOUT
}ow_error_t;

temp_stage_t temp_stage = TEMP_WAITING;

int onewire_doing_reset = 0;
int onewire_current_bit_n = 0;
int onewire_current_byte_n = 0;
int onewire_current_byte = 0;
int onewire_total_bytes = 0;
ow_error_t onewire_latest_error = ERR_NONE;
uint8_t onewire_buffer[16] = {};

void onewire_on_transfer_complete();



void uart_set_baudrate(int rate){
    USART1_BRR = (96000000 + rate / 2) / rate;
}


void init_uart(){
    USART1_CR1 |= USART_CR1_UE;
    uart_set_baudrate(115200);
    USART1_CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART1_CR3 |= USART_CR3_HDSEL;
}

void init_timer(){
    NVIC_ISER(NVIC_TIM2_IRQ / 32) = 1 << (NVIC_TIM2_IRQ % 32);

    TIM2_PSC = 10000; // frq = 9600 hz
    TIM2_ARR = 9600/2; // interrupt frq = 2 hz;
    TIM2_DIER |= TIM_DIER_UIE;
    TIM2_CR1 |= TIM_CR1_CEN;
}

void init_thermometer(){
    init_uart();
    init_timer();
}

void usart1_isr(){
    if(!(USART1_SR & USART_SR_RXNE)){
        return; // wrong interrupt
    }
    int was_reset = onewire_doing_reset;
    uint8_t data_received = USART1_DR;
    if(onewire_doing_reset){
        uart_set_baudrate(115200);
        onewire_doing_reset = 0;
        if(data_received == 0xf0){
            onewire_latest_error = ERR_NO_DEVICE;
            NVIC_ICER(NVIC_USART1_IRQ / 32) = 1 << (NVIC_USART1_IRQ % 32); // disable interrupt
            onewire_on_transfer_complete();
            return;
        }
        // TODO: check crc
    }
    int finished = 0;

    if(!was_reset){
        if(data_received == 0xff) {
            onewire_current_byte |= (1 << onewire_current_bit_n);
        }
        onewire_current_bit_n++;
        if(onewire_current_bit_n == 8){
            onewire_current_bit_n = 0;
            onewire_buffer[onewire_current_byte_n] = onewire_current_byte;
            onewire_current_byte = 0;
            onewire_current_byte_n++;
            if(onewire_current_byte_n == onewire_total_bytes){
                // end of transfer
                NVIC_ICER(NVIC_USART1_IRQ / 32) = 1 << (NVIC_USART1_IRQ % 32);
                onewire_on_transfer_complete();
                finished = 1;
            }
        }
    }

    if(!finished){
        USART1_DR = ((onewire_buffer[onewire_current_byte_n] >> onewire_current_bit_n) & 0x01) ? 0xff : 0x00;
    }
}

void onewire_do_async(int len){
    onewire_doing_reset = 1;
    onewire_current_bit_n = 0;
    onewire_current_byte_n = 0;
    onewire_current_byte = 0;
    onewire_total_bytes = len;
    // send reset pulse
    uart_set_baudrate(9600);
    USART1_SR &= ~(USART_SR_RXNE);
    NVIC_ISER(NVIC_USART1_IRQ / 32) = 1 << (NVIC_USART1_IRQ % 32);
    USART1_DR = 0xf0;
}

// wait -> ram -> config -> convert -> wait -> ...
void tim2_isr(){
    TIM2_SR = ~TIM_SR_UIF;
    // runs twice per second
    if(temp_stage != TEMP_WAITING){
        // TODO: this won't work
        onewire_latest_error = ERR_TIMEOUT;
    }

    temp_stage = TEMP_READING_RAM;
    onewire_buffer[0] = 0xcc;
    onewire_buffer[1] = 0xbe;
    memset(onewire_buffer + 2, 0xff, 9);
    onewire_do_async(11);
}

void onewire_on_transfer_complete(){
    if(temp_stage == TEMP_READING_RAM){
        // print temp
        if(onewire_latest_error != ERR_NONE){
            matrix_set_character(character_E, 0);
            switch (onewire_latest_error) {
                case ERR_NO_DEVICE:
                    matrix_set_character(character_D, 1);
                    break;
                case ERR_TIMEOUT:
                    matrix_set_character(character_T, 1);
            }
            onewire_latest_error = ERR_NONE;
            matrix_flush();
        } else{
            uint16_t temp = *((uint16_t *)(onewire_buffer + 2));
            print_num(temp / 16, (temp / 2) & 0b111);
        }

        // send config
        onewire_buffer[0] = 0xcc;
        onewire_buffer[1] = 0x4e;
        onewire_buffer[2] = 127; // dummy
        onewire_buffer[3] = 127; // dummy
        onewire_buffer[4] = 0b01000000;
        temp_stage = TEMP_SENDING_CONFIG;
        onewire_do_async(5);

    } else if(temp_stage == TEMP_SENDING_CONFIG){
        // start next conversion
        onewire_buffer[0] = 0xcc;
        onewire_buffer[1] = 0x44;
        temp_stage = TEMP_SENDING_CONVT;
        onewire_do_async(2);
    } else if (temp_stage == TEMP_SENDING_CONVT) {
        temp_stage = TEMP_WAITING;
    }
}

