#include <iostream>
#include <stdint.h>
#include "motor_utils.h"

// TODO: Get the pru values to update somewhere

uint32_t *pru_mem;

int main(int, char**){
    // Get an instance of pru 1's memory
    pru_mem = get_pru1_array();
    if (pru_mem == nullptr) {
        return 1;
    }

    // Close the PRU's memory
    close_pru1_mem(pru_mem);

    return 0;
}
