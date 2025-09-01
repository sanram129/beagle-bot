#include "leg.h"

// Constructor
Leg::Leg(int8_t leg_loc) {
    // Set IK angles
    this->ik_angles[0] = 0.0;
    this->ik_angles[1] = 3*M_PI/4;
    this->ik_angles[2] = M_PI/2;

    // Set default motor angles to 0;
    this->motor_angles[0] = 0.0;
    this->motor_angles[1] = 0.0;
    this->motor_angles[2] = 0.0;

    // Set which leg this actually is
    this->leg_loc = leg_loc;
}

float *Leg::ik_to_motor(float *ik_angles) {
    float *new_motor_angles = (float*) malloc(3*sizeof(float));

    // TODO

    return new_motor_angles;
}

void Leg::set_ee(float *position) {
    // Make sure we have 3 positions
    int pos_space = sizeof(position) / sizeof(position[0]);
    if (pos_space != 3) {
        std::cout << "Not enough position arguments!" << std::endl;
        return;
    }

    float tmp_angles[3] = {0.0};

    // Get the angle made by the first motor and make sure it is valid
    double yz_dist = sqrt(pow(position[1], 2) + pow(position[2], 2));
    tmp_angles[0] = acos(LEG_LINK_1_LEN/yz_dist) - acos(position[1]/yz_dist);
    if (tmp_angles[0] > M_PI/2 || tmp_angles[0] < -M_PI/2) {
        std::cout << "Invalid end effector position!" << std::endl;
        return;
    }

    // Calculate the z coordinate of joint 2 to get distances
    float joint_2 = LEG_LINK_1_LEN*sin(tmp_angles[0]);
    float j2_ee_z = position[2] - joint_2;

    // Get the remaining two angles now that they are coplanar with the end effector
    double j2_ee = sqrt(pow((position[0]), 2) + pow(j2_ee_z, 2));
    
    tmp_angles[2] = acos((pow(LEG_LINK_2_LEN, 2) + pow(LEG_LINK_3_LEN, 2) - pow(j2_ee, 2))
                        / 2*LEG_LINK_2_LEN*LEG_LINK_3_LEN);

    tmp_angles[1] = M_PI/2 + asin(LEG_LINK_3_LEN*sin(tmp_angles[2])/j2_ee) + acos(j2_ee_z/j2_ee);

    if (tmp_angles[1] > M_PI || tmp_angles[1] < M_PI/4) {
        std::cout << "Invalid end effector position!" << std::endl;
        return;
    }

    if (tmp_angles[2] > M_PI || tmp_angles[2] < 0) {
        std::cout << "Invalid end effector position!" << std::endl;
        return;
    }
    
    // If code gets here all is valid, update coords
    float *new_motor_angles = ik_to_motor(tmp_angles);

    for (int i = 0; i < 3; i++) {
        this->ik_angles[i] = tmp_angles[i];
        this->motor_angles[i] = new_motor_angles[i];
        this->end_effector[i] = position[i];
    }

    free(new_motor_angles);
}

float *Leg::get_ee() { return this->end_effector; }

float *Leg::get_ik_angles() { return this->ik_angles; }

float *Leg::get_motor_angles() { return this->motor_angles; }