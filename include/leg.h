#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <cstdlib>

#define LEG_LINK_1_LEN 16.5
#define LEG_LINK_2_LEN 80.0
#define LEG_LINK_3_LEN 80.0

enum leg_locations {
    FRONT_LEFT,
    FRONT_RIGHT,
    BACK_LEFT,
    BACK_RIGHT
};

class Leg {
    public:
        Leg(int8_t leg_loc);

        // Convert between IK and motor angles
        float *ik_to_motor(float *ik_angles);

        // Setter and getter for end effector
        void set_ee(float *position);
        float *get_ee();
        
        // Getter for angles
        float *get_ik_angles();

        // Getter for motor
        float *get_motor_angles();

    private:
        float ik_angles[3];
        float motor_angles[3];
        float end_effector[3];
        int8_t leg_loc;
};