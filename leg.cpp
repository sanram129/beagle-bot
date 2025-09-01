#include "leg.h"

// Constructor
Leg::Leg(int* pos_offsets) {
    // Set IK angles
    this->ik_angles[0] = 0.0;
    this->ik_angles[1] = M_PI/2;
    this->ik_angles[2] = M_PI/2;

    // Set default motor angles to 0
    // Also sets the positive axis offsets of the current leg
    for (int i = 0; i < 3; i++) {
        this->motor_angles[i] = 0.0;
        this->pos_offsets[i] = pos_offsets[i];
    }

    // Set end effector to its home position
    this->end_effector[0] = 0.0;
    this->end_effector[1] = 16.5;
    this->end_effector[2] = -sqrt(2)*80.0;
}

float *Leg::ik_to_motor(float *ik_angles) {
    float* new_motor_angles = (float*) malloc(sizeof(float)*3);

    // The list of radian offsets of each of the motors
    float rad_offsets[] = {0.0, -M_PI/2, -M_PI/2};

    // Take away offsets and multiply by the remainder
    for (int i = 0; i < 3; i++) {
        new_motor_angles[i] = this->ik_angles[i];
        new_motor_angles[i] -= rad_offsets[i];
        new_motor_angles[i] *= this->pos_offsets[i]; 
    }

    return new_motor_angles;
}

void Leg::set_ee(float *position) {
    // Clamp all the ee values
    position[0] = std::max((float)-160.0, std::min(position[0], (float)160.0));
    position[1] = std::max((float)-160.0, std::min(position[1], (float)160.0));
    position[2] = std::max((float)-60.0, std::min(position[0], (float)-160.0));

    float tmp_angles[3] = {0.0};

    // Get the angle made by the first motor and make sure it is valid
    double yz_dist = sqrt(pow(position[1], 2) + pow(position[2], 2));
    tmp_angles[0] = acos(LEG_LINK_1_LEN/yz_dist) - acos(position[1]/yz_dist);
    //tmp_angles[0] = std::max((float)-M_PI/3, std::min(tmp_angles[0], (float)M_PI/3));

    // Calculate the z coordinate of joint 2 to get distances
    float joint_2 = LEG_LINK_1_LEN*sin(tmp_angles[0]);
    float j2_ee_z = position[2] - joint_2;

    // Get the remaining two angles now that they are coplanar with the end effector
    double j2_ee = sqrt(pow(position[0], 2) + pow(j2_ee_z, 2));

    std::cout << j2_ee << std::endl;

    tmp_angles[2] = acos((pow(LEG_LINK_2_LEN, 2) + pow(LEG_LINK_3_LEN, 2) - pow(j2_ee, 2))
                        / 2.0*LEG_LINK_2_LEN*LEG_LINK_3_LEN);

    tmp_angles[1] = M_PI/4.0 + asin(LEG_LINK_3_LEN*sin(tmp_angles[2])/j2_ee) + acos(j2_ee_z/j2_ee);

    if (tmp_angles[1] > M_PI || tmp_angles[1] < 0) {
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

float *Leg::get_ee() { 
    float* tmp_ee = (float*) malloc(sizeof(float)*3);

    for (int i = 0; i < 3; i++) {
        tmp_ee[i] = this->end_effector[i];
    }

    return tmp_ee; 
}

float *Leg::get_ik_angles() {
    float* tmp_ika = (float*) malloc(sizeof(float)*3);

    for (int i = 0; i < 3; i++) {
        tmp_ika[i] = this->ik_angles[i];
    }

    return tmp_ika;
}

float *Leg::get_motor_angles() {
    float* tmp_ma = (float*) malloc(sizeof(float)*3);

    for (int i = 0; i < 3; i++) {
        tmp_ma[i] = this->motor_angles[i];
    }

    return tmp_ma;
}

void Leg::leg_test(int axis, float delta) {
    // Copy the current end effector position
    float new_pos[3] = {this->end_effector[0], this->end_effector[1], this->end_effector[2]};

    // Update based on desired axis
    new_pos[axis] += delta;
    this->set_ee(new_pos);
}

Leg::~Leg() {
    free(this->pos_offsets);
}
