#pragma once

// matrix
void matrix_set_character(const uint8_t *data, int place);
void print_num(int n, int bar);
void matrix_flush();
void matrix_init();

extern uint8_t character_D[3];
extern uint8_t character_E[3];
extern uint8_t character_T[3];