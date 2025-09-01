#include <cmath>

#include "leg.h"

class Robot {
    public:
        Robot();

        // Make the new standing height of the robot a certain value
        void set_home_height(float height);

        // Make the robot walk forward
        float **walk(float dx, float dy);
    private:
        Leg legs[4];
        float goal_angles[12];
};