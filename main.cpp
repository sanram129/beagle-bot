#include <iostream>
#include <stdint.h>
#include <linux/joystick.h>
#include <cstdlib>
#include <cmath>
#include <thread>

#include "motor_utils.h"
#include "beagle_robot.h"

// The robot
Robot *beagle_bot;

// Stores the pru array of cycles
uint32_t *pru_mem;

// Initialize arrays for holding the most recent useful controller input
int axes[3][2] = {
    {0, 0},     // Left joystick, left-right and up-down
    {0, 0},     // Right joystick, left-right and up-down
    {0, 0}      // Triggers, left and right
};

// Joystick file
int joy;

// Event handler for joystick
struct js_event ev;

// Keep the current target position in an array
float *current_target_position = new float[12];

// Send the final cycles to the PRU
void send_to_pru(float *motor_angles) {
    // Send the cycle amounts to the PRU
    for (int i = 0; i < 12; i++) {
        pru_mem[i] = degrees_to_cycles(motor_angles[i]);
    }
}

// Handle joystick input to not clog the main thread (allows for smoother presses)
void joy_handler() {
    while (1) {
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
    }
}

int main(int, char**){
    // Initialize the robot object
    beagle_bot = new Robot();

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
    joy = open("/dev/input/js0", O_RDONLY);
    if (joy == -1) {
        std::cout << "Joystick not opened!!!" << std::endl;
        return 1;
    }

    // Create joystick handler thread
    std::thread joy_thread(joy_handler);

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

        try {
            // Store the potential new robot position
            float height_delta = 0.0;

            // Change height and crouch first
            // Right trigger goes up, left goes down
            if (axes[2][0] != 0 || axes[2][1] != 0) {
		        height_delta = (axes[2][1] - axes[2][0])/3276700.0/2;
		        beagle_bot->crouch(height_delta);
            } else {
                // Get dx and dy in range of -1 to 1
                float dx = axes[0][0]/32767.0;
                float dy = axes[0][1]/32767.0;

                // Walk if we don't want to crouch
                beagle_bot->walk(dx, dy);
            }
		
            // Update target
            float *temp_target = beagle_bot->get_goal_angles();
            for (int i = 0; i < 12; i++) {
                current_target_position[i] = temp_target[i]*180.0/(float)M_PI;
            }

            free(temp_target);

        } catch (int e) {
            std::cout << "Error occurred: " << e << std::endl;
            break;
	}

        // Push the new array to the PRU
        send_to_pru(current_target_position);
    }

    // Bring thread back with main
    joy_thread.join();

    // Close the PRU's memory
    close_pru1_mem(pru_mem);

    return 0;
}
