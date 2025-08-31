#include "motor_utils.h"

uint32_t *get_pru1_array() {
    
    // Pointer to memory
    uint32_t *pru_mem;

    // Open memory file
    int mem_file = open("/dev/mem", O_RDWR | O_SYNC);

    // Return a null pointer if there is an error
    if (mem_file == -1) {
        std::cout << "/dev/mem not opened!" << std::endl;
        return nullptr;
    }

    // Map the pru's memory 
    pru_mem = (uint32_t *) mmap(0, PRU_MEM_LEN, PROT_READ | PROT_WRITE, MAP_SHARED, mem_file, PRU_ADDR);

    // Return a nullptr if error occurs in mapping
    if (pru_mem == MAP_FAILED) {
        std::cout << "pru memory not mapped!" << std::endl;
        return nullptr;
    }

    close(mem_file);

    // Add 0x200 to skip stack and heap; divide by 4 because int32
    return (uint32_t *) (pru_mem + PRU1_OFFSET/4 + 0x200/4); 
}

float cycles_to_degrees(uint32_t cycles) {
    return (0.0117 * (cycles - 100000/26.0) - 90.0);
}

uint32_t degrees_to_cycles(float degrees) {
    return (uint32_t) (1/0.0117 * (degrees + 90) + 100000/26.0);
}

float *get_motor_angles(uint32_t *pru1_mem) {
    float *angles = (float *) malloc(sizeof(float)*12);

    // Check for a malloc fail
    if (angles == nullptr) {
        std::cout << "angles array not allocated!" << std::endl;
        return angles;
    }

    // Loop through and change the values in the array
    for (int i = 0; i < 12; i++) {
        angles[i] = cycles_to_degrees(pru1_mem[i]);
    }

    return angles;
}

void update_pru_vals(uint32_t *pru1_mem, float *cycles) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        pru1_mem[i] = degrees_to_cycles(cycles[i]);
    }
}

void close_pru1_mem(uint32_t *pru1_mem) {
    if (munmap(pru1_mem, PRU_MEM_LEN)) {
        std::cout << "memory unmap failed!" << std::endl;;
    }
}