#include <iostream>
#include <stdint.h>
#include <linux/joystick.h>
#include <cstdlib>

#include "motor_utils.h"
#include "leg.h"

// TODO: test a single leg with IK
Leg *test_leg;

// Stores the pru array of cycles
uint32_t *pru_mem;

// Keep the current target position in an array
float *current_target_position = new float[12];

// Send the final cycles to the PRU
void send_to_pru(float *motor_angles) {
    // Send the cycle amounts to the PRU
    for (int i = 0; i < 12; i++) {
        pru_mem[i] = degrees_to_cycles(motor_angles[i]);
    }
}

int main(int, char**){
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

    // Handle motor angle updates in the background while the main thread makes calculations
    std::thread pru_sender(send_to_pru, goal_deg_queue);

    // Event handler for joystick
    struct js_event ev;

    // Test leg initialization
    int pos_offsets[] = {-1, -1, 1};
    test_leg = new Leg(pos_offsets);

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
            // Store the potential new end effector position
            float delta = 0.0;

            int leg_axis = -1;

	        read(joy, &ev, sizeof(ev));
            switch((int)ev.type) {
                case JS_EVENT_BUTTON:
                    break;
                case JS_EVENT_AXIS:
                    if ((int)ev.number == 1) { // x
                        delta = (int)ev.value * 10.0 / 32767.0;
                        leg_axis = 0;
                    } else if ((int)ev.number == 0) {// y
                        delta = (int)ev.value * 10.0 / 32767.0;
                        leg_axis = 1;
                    } else if ((int)ev.number == 3) { // z
                        delta = (int)ev.value * 10.0 / 32767.0;
                        leg_axis = 2;
		    }
                    break;
                default:
                    break;
            }

            if (leg_axis != -1) {
                // Do IK on the new leg
                test_leg->leg_test(leg_axis, delta);

                // Get the new motor angles
                float *new_motor = test_leg->get_motor_angles();

                // Test front right leg
                current_target_position[6] = new_motor[0];
                current_target_position[6] = new_motor[1];
                current_target_position[6] = new_motor[2];

                free(new_motor);
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

    // Join the thread
    pru_sender.join();

    return 0;
}
