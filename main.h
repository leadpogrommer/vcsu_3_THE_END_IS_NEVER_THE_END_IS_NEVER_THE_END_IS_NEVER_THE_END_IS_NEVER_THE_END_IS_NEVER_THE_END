#pragma once

#include "stdint.h"

// matrix
void matrix_set_character(const uint8_t *data, int place);
void print_num(int n, int bar);
void matrix_flush();
void matrix_init();

extern uint8_t character_D[];
extern uint8_t character_E[];
extern uint8_t character_T[];
extern uint8_t font[][3];


// thermometer
void init_thermometer();