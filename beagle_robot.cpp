#include "beagle_robot.h"

Robot::Robot() {
    // The offsets required to align all axes
    int offsets[4][3] = {
        {1, 1, -1},
        {-1, -1, 1}, // was -1 1 -1
        {-1, 1, -1}, // was -1 -1 1
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
    if (this->home_height <= -60.0 && this->home_height >= -150.0)
	this->home_height += delta;
    else
	this->home_height = (float)std::min(-60.0, std::max((double)this->home_height, -150.0));

    // Get the new motor angles for each of the legs and change the array
    for (int i = 0; i < 4; i++) {
        float *old_ee = this->legs[i]->get_ee();
	    old_ee[2] = this->home_height; 
        this->legs[i]->set_ee(old_ee);
    }

    // Update goal angles!
    for (int i = 0; i < 4; i++) {
        float *tmp_motor_angles = this->legs[i]->get_motor_angles();

        for (int j = 0; j < 3; j++) {
            this->goal_angles[3*i + j] = tmp_motor_angles[j];
        }

        free(tmp_motor_angles);
    }
}

void Robot::stride(float normal_dx, float normal_dy, float dist) {
    // Simply switch the current pose and set the angles to the one that matches
    // Front left and back right move up for the "1" sequence
    float *end_effectors[4] = {
        this->legs[FRONT_LEFT]->get_ee(),
        this->legs[FRONT_RIGHT]->get_ee(),
        this->legs[BACK_LEFT]->get_ee(),
        this->legs[BACK_RIGHT]->get_ee()
    };

    switch (this->current_pose) {
        case STEP_UP_1: 
            // Move front left and back right in desired direction by distance and up
            end_effectors[FRONT_LEFT][0] += normal_dy*dist;
            end_effectors[FRONT_LEFT][1] += normal_dx*dist;
            end_effectors[FRONT_LEFT][2] -= dist;

            end_effectors[BACK_RIGHT][0] += normal_dy*dist; 
            end_effectors[BACK_RIGHT][1] -= normal_dx*dist; // Right side has an inverted y-axis so we can negate it
            end_effectors[BACK_RIGHT][2] -= dist;

            // Move front right and back left backwards by same offset; stay on the ground
            end_effectors[FRONT_RIGHT][0] -= normal_dy*dist;
            end_effectors[FRONT_RIGHT][1] += normal_dx*dist;

            end_effectors[BACK_LEFT][0] -= normal_dy*dist;
            end_effectors[BACK_LEFT][1] -= normal_dx*dist;
        
            break;
        case STEP_DOWN_1:
            // Move front left and back right in desired direction by distance and down 
            end_effectors[FRONT_LEFT][0] += normal_dy*dist;
            end_effectors[FRONT_LEFT][1] += normal_dx*dist;
            end_effectors[FRONT_LEFT][2] += dist;

            end_effectors[BACK_RIGHT][0] += normal_dy*dist;
            end_effectors[BACK_RIGHT][1] -= normal_dx*dist; // Right side has an inverted y-axis so we can negate it
            end_effectors[BACK_RIGHT][2] += dist;

            // Move front right and back left backwards by same offset; stay on the ground
            end_effectors[FRONT_RIGHT][0] -= normal_dy*dist;
            end_effectors[FRONT_RIGHT][1] += normal_dx*dist;

            end_effectors[BACK_LEFT][0] -= normal_dy*dist;
            end_effectors[BACK_LEFT][1] -= normal_dx*dist;
        
            break;
        case STEP_UP_2:
            // Move front right and back left in desired direction by distance and up 
            end_effectors[FRONT_RIGHT][0] += normal_dy*dist;
            end_effectors[FRONT_RIGHT][1] -= normal_dx*dist;
            end_effectors[FRONT_RIGHT][2] -= dist;

            end_effectors[BACK_LEFT][0] += normal_dy*dist;
            end_effectors[BACK_LEFT][1] += normal_dx*dist;
            end_effectors[BACK_LEFT][2] -= dist;

            // Move front left and back right backwards by same offset; stay on the ground
            end_effectors[FRONT_LEFT][0] -= normal_dy*dist;
            end_effectors[FRONT_LEFT][1] -= normal_dx*dist;

            end_effectors[BACK_RIGHT][0] -= normal_dy*dist;
            end_effectors[BACK_RIGHT][1] += normal_dx*dist;
        
            break;
        case STEP_DOWN_2:
            // Move front right and back left in desired direction by distance and down
            end_effectors[FRONT_RIGHT][0] += normal_dy*dist;
            end_effectors[FRONT_RIGHT][1] -= normal_dx*dist;
            end_effectors[FRONT_RIGHT][2] += dist;

            end_effectors[BACK_LEFT][0] += normal_dy*dist;
            end_effectors[BACK_LEFT][1] += normal_dx*dist;
            end_effectors[BACK_LEFT][2] += dist;

            // Move front left and back right backwards by same offset; stay on the ground
            end_effectors[FRONT_LEFT][0] -= normal_dy*dist;
            end_effectors[FRONT_LEFT][1] -= normal_dx*dist;

            end_effectors[BACK_RIGHT][0] -= normal_dy*dist;
            end_effectors[BACK_RIGHT][1] += normal_dx*dist;
        
            break;
        default:
            break;
    }

    // Now with the end effectors all with correct values, update everything
    for (int i = 0; i < 4; i++) {
        this->legs[i]->set_ee(end_effectors[i]);
    }

    // Update goal angles!
    for (int i = 0; i < 4; i++) {
        float *tmp_motor_angles = this->legs[i]->get_motor_angles();

        for (int j = 0; j < 3; j++) {
            this->goal_angles[3*i + j] = tmp_motor_angles[j];
        }

        free(tmp_motor_angles);
    }
}

void Robot::walk(float dx, float dy) {
    // Normalize dx and dy as a single vector
    float mag = sqrt(pow(dx, 2) + pow(dy, 2));
    dx /= mag;
    dy /= mag;

    // Check if walking cycle finished; if not, move to next pose
    if (this->current_pose == STEP_DOWN_2) {
        this->current_pose = STAND;
        return;
    } else {
        this->current_pose++;
    }

    // Take a stride and compute the angles
    this->stride(dx, dy, 25);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
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
