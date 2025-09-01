#include <cmath>
#include <cstdlib>

#include "leg.h"

enum leg_pos {
    FRONT_LEFT = 0,
    BACK_LEFT,
    FRONT_RIGHT,
    BACK_RIGHT
};

enum poses {
    STAND = 0,
    STEP_UP_1, 
    STEP_DOWN_1,
    STEP_UP_2,
    STEP_DOWN_2
};

class Robot {
    public:
        Robot();

        // Make the new standing height of the robot a certain value and hold it
        void crouch(float delta);

        // Make the robot walk forward and return the desired pose
        void walk(float dx, float dy);

        // Get the goal angles for the current pose
        float *get_goal_angles();
    private:
        Leg legs[4];
        float goal_angles[12]; // Motor angles
        float home_height;
        int current_pose;
};