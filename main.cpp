#include <iostream>
#include <stdint.h>
#include <linux/joystick.h>
#include <thread>
#include <vector>

#include "motor_utils.h"
#include "beagle_robot.h"

// TODO:

// Create variables
uint32_t *pru_mem;

// Keep all the most recent target positions in a vector as a queue
std::vector<float *> goal_deg_queue;

// Send the final cycles to the PRU
void send_to_pru(std::vector<float *> motor_angles) {
    for (int i = 0; i < 12; i++) {
        pru_mem[i] = degrees_to_cycles(motor_angles[0][i]);
    }

    // Remove the first if there is more than one to process
    if (motor_angles.size() > 1)
        goal_deg_queue.erase(goal_deg_queue.begin());
}

int main(int, char**){
    // Get an instance of pru 1's memory
    pru_mem = get_pru1_array();
    if (pru_mem == nullptr) {
        return 1;
    }

    // Open the joystick
    int joy = open("/dev/input/js0", O_RDONLY);
    if (joy == -1) {
        std::cout << "Joystick not opened!!!" << std::endl;
        return 1;
    }

    // Handle motor angle updates in the background while the main thread makes calculations
    std::thread pru_sender(send_to_pru, goal_deg_queue);

    // Main loop for input and inverse kinematics/walk
    while (1) {

    }

    // Close the PRU's memory
    close_pru1_mem(pru_mem);

    // Terminate all threads
    std::terminate();

    return 0;
}
