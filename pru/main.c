#include <stdint.h>
#include <pru_cfg.h>
#include "resource_table_empty.h"

// Open the RAM of PRU 1
volatile unsigned int *pru1_dram = (unsigned int *) 0x200;

// Servo control function (explanation in demos)
extern void servo_duty(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t,
                       uint32_t, uint32_t, uint32_t, uint32_t, uint32_t,
                       uint32_t, uint32_t, uint32_t);

int main(void)
{
    // Create an array to get the values at the current time
    unsigned int saved_values[12] = {0};

    // Initialize the pru's memory and saved values to store all angles at 90
    uint8_t servo;
    for (servo = 0; servo < 12; servo++) {
        pru1_dram[servo] = 11538;
    }

    // Enable OCP master port
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    while (1) {
        // Take a snapshot of the values at this time
        for (servo = 0; servo < 12; servo++) {
            saved_values[servo] = pru1_dram[servo];
        }

        // Push the values to the assembly function
        servo_duty(153846,
                   saved_values[0], saved_values[1], saved_values[2], saved_values[3],
                   saved_values[4], saved_values[5], saved_values[6], saved_values[7],
                   saved_values[8], saved_values[9], saved_values[10], saved_values[11]);

    }

    // Shouldn't be reached, but kills the processes on the PRU
    __halt();

	return 0;
}
