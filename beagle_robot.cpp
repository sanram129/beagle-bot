#include "beagle_robot.h"

Robot::Robot() {
    // The offsets required to align all axes
    int offsets[4][3] = {
        {1, 1, -1},
        {-1, 1, -1},
        {-1, -1, 1},
        {1, -1, 1}
    };

    for (int i = 0; i < 4; i++) {
	    int *leg_offset = new int[3];
	    for (int j = 0; j < 3; j++) {
	        leg_offset[j] = offsets[i][j];
	    }
        this->legs[i] = new Leg(leg_offset);
    }

    this->home_height = -sqrt(2)*80.0;

    this->current_pose = STAND;
}

void Robot::crouch(float delta) {
    // Only do this if current position is standing
    if (this->current_pose != STAND)
        return;

    // Change home height (-ve for taller, +ve for shorter)
    this->home_height += delta;

    // Get the new motor angles for each of the legs and change the array
    for (int i = 0; i < 4; i++) {
        float *old_ee = this->legs[i]->get_ee();
        old_ee[2] += delta; 
        this->legs[i]->set_ee(old_ee);
        free(old_ee);
    }

    // Update goal angles!
    for (int i = 0; i < 4; i++) {
        float *tmp_motor_angles = this->legs[i]->get_ee();
        
        for (int j = 0; j < 3; j++) {
            this->goal_angles[3*i + j] = tmp_motor_angles[j];
        }

        free(tmp_motor_angles);
    }
}

void Robot::walk(float dx, float dy) {

}

float *Robot::get_goal_angles() {
    float *tmp_goal_angles = (float*) malloc(sizeof(float) * 12);

    for (int i = 0; i < 12; i++) {
        tmp_goal_angles[i] = this->goal_angles[i];
    }

    return tmp_goal_angles;
}

Robot::~Robot() {
    for (int i = 0; i < 4; i++) {
        free(this->legs[i]);
    }
}