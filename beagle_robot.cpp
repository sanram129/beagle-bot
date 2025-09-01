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
        this->legs[i] = Leg(offsets[i]);
    }

    this->home_height = sqrt(2)*80.0;

    this->current_pose = STAND;
}

void Robot::crouch(float delta) {
    // Only do this if current position is standing
    if (this->current_pose != STAND)
        return;

    // Change home height
    this->home_height += delta;

    // Get the new motor angles for each of the legs and change the array
    for (int i = 0; i < 4; i++) {
        float *old_ee = this->legs[i].get_ee();
        old_ee[2] += delta; 
        this->legs[i].set_ee(old_ee);
        free(old_ee);
    }

    // TODO: Update goal angles!
}

void Robot::walk(float dx, float dy) {

}

float *get_goal_angles() {

}