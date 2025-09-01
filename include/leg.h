#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <cstdlib>

#define LEG_LINK_1_LEN 16.5
#define LEG_LINK_2_LEN 80.0
#define LEG_LINK_3_LEN 80.0

enum leg_locations {
    FRONT_LEFT = 0,
    BACK_LEFT,
    FRONT_RIGHT,
    BACK_RIGHT
};

enum axes {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS
};

class Leg {
    public:
        Leg(int *pos_offsets); 

        // Convert between IK and motor angles
        float *ik_to_motor(float *ik_angles);

        // Setter and getter for end effector
        void set_ee(float *position);
        float *get_ee();

        // Getter for angles
        float *get_ik_angles();

        // Getter for motor
        float *get_motor_angles();

        // Move the leg along an axis
        void leg_test(int axis, float delta);

    private:
        float ik_angles[3];
        float motor_angles[3];
        float end_effector[3];
        int pos_offsets[3];
};
