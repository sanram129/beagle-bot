#include <iostream>
#include <stdint.h>
#include <linux/joystick.h>
#include <cstdlib>
#include <cmath>

#include "motor_utils.h"
#include "beagle_robot.h"

// Stores the pru array of cycles
uint32_t *pru_mem;

// Keep the current target position in an array
float *current_target_position = new float[12];

// Send the final cycles to the PRU
void send_to_pru(float *motor_angles) {
    //std::cout << "Target positions are: " << current_target_position[3]
	//<< " " << current_target_position[4] << " " << current_target_position[5] << std::endl;
    // Send the cycle amounts to the PRU
    for (int i = 0; i < 12; i++) {
        pru_mem[i] = degrees_to_cycles(motor_angles[i]);
    }
}

int main(int, char**){
    // Initialize the robot object
    Robot beagle_bot();

    // Initialize target positions
    for (int i = 0; i < 12; i++) {
        current_target_position[i] = 0.0;
    }

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

    // Event handler for joystick
    struct js_event ev;

    // Initialize arrays for holding the most recent useful controller input
    int axes[][] = {
        {0, 0},     // Left joystick, left-right and up-down
        (0, 0),     // Right joystick, left-right and up-down
        {0, 0}      // Triggers, left and right
    };

    // Main loop is ready for input
    std::cout << "Init complete." << std::endl;

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

	    Axis 4 = Right trigger
		Down = 32767
		Up (Not pressed) = -32767

	    Axis 5 = Left trigger
		Down = 32767
		Up (Not pressed) = -32767

	    BUTTONS

	    Button 6 = Left bumper
	    Button 7 = Right bumper
	*/

        // Read the input from joy
        try {
            // Store the potential new robot position
            float height_delta = 0.0;

	        read(joy, &ev, sizeof(ev));
            switch((int)ev.type) {
                case JS_EVENT_BUTTON:
                    break;
                case JS_EVENT_AXIS:
                    switch ((int)ev.number) {
                        case 0:
                        case 1:
                        case 2:
                        case 3:
                            // Switch all axes so up and left is positive
                            axes[(int)ev.number/2][(int)ev.number % 2] = (int)ev.value;
                            break;
                        case 4:
                        case 5:
                            axes[2][(int)ev.number % 2] = ((int)ev.value + 32767) / 2;
                            break;
                    }
                    break;
                default:
                    break;
            }

            // Change height and crouch first
            // Right trigger goes up, left goes down
            if (axes[2][0] != 0 || axes[2][1] != 0) {
                height_delta = (axes[2][1] - axes[2][0])/32767.0;
                beagle_bot.crouch(height_delta);
            } else {
                // Walk if we don't want to crouch
            }

            // Update target
            float *temp_target = beagle_bot.get_goal_angles();
            for (int i = 0; i < 12; i++) {
                current_target_position[i] = temp_target[i];
            }

        } catch (int e) {
            std::cout << "Error occurred: " << e << std::endl;
            break;
	    }

        // Push the new array to the PRU
        send_to_pru(current_target_position);
    }

    // Close the PRU's memory
    close_pru1_mem(pru_mem);

    return 0;
}
