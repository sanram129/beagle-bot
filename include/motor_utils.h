// Utilities to facilitate the updating of duty cycles in the PRU's memory

#pragma once

#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdlib.h>

#define ANGLE_MIN -90
#define ANGLE_MAX 90

#define MOTOR_COUNT 12

#define PRU_ADDR 0x4A300000
#define PRU1_OFFSET 0x02000
#define PRU_MEM_LEN 0x80000

// Returns a pointer to the array of pru values stored in memory
uint32_t *get_pru1_array();

// Converts the number of "on" clock cycles to degrees
float cycles_to_degrees(uint32_t cycles);

// Converts the number of degrees to "on" cycles
uint32_t degrees_to_cycles(float degrees);

// Returns an array of the current motor angles in degrees
float *get_motor_angles(uint32_t *pru1_mem);

// Update the motor values in the pru memory
void update_pru_vals(uint32_t *pru1_mem, float *cycles);

// Closes the pru's memory when done 
void close_pru1_mem(uint32_t *pru1_mem);
