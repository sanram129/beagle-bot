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
    // Do nothing if there are no packages to send
    if (motor_angles.size() == 0)
	return;

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
    //std::thread pru_sender(send_to_pru, goal_deg_queue);

    // Event handler for joystick
    struct js_event ev;

    // Main loop for input and inverse kinematics/walk
    while (1) {
	/*
	    JOYSTICK BINDINGS

	    AXES

	    Axis 0 = Left joystick
		Left = -32767
		Right = 32767

	    Axis 1 = Left joystick
		Down = 32767
		Up = -32767

	    Axis 2 = Right joystick
		Left = -32767
		Right = 32767

	    Axis 3 = Right joystick
		Down = 32767
		Up = -32767

	    BUTTONS

	    Button 6 = Left bumper
	    Button 7 = Right bumper
	*/

        // Read the input from joy
        read(joy, &ev, sizeof(ev));
        switch((int)ev.type) {
            case JS_EVENT_BUTTON:
		std::cout << "Button pressed!" << std::endl;
                std::cout << "Button event number: " << (int)ev.number << std::endl;
                std::cout << "Button event value: " << (int)ev.value << std::endl;
                break;
            case JS_EVENT_AXIS:
		if ((int)ev.number == 3)
			std::cout << (int)ev.value << std::endl;
                break;
            default:
                break;
        }
    }

    // Close the PRU's memory
    close_pru1_mem(pru_mem);

    // Join the thread
    //pru_sender.join();

    return 0;
}
