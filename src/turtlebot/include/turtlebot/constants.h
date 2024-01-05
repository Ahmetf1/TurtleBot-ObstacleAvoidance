/*
Date: 19.12.2023
Developed by: Ahmet Furkan Akıncı
Project: EE 451 - Project 4
Summary: Contains constants 
*/

#ifndef TURTLEBOT_CONSTANTS
#define TURTLEBOT_CONSTANTS

#define _USE_MATH_DEFINES

#include <math.h>
#include <turtlebot/Turtlebot.h>
#include <turtlebot/ApfPlanner.h>

namespace Turtlebot {
    namespace constants {
        constexpr double control_rate = 30;
        constexpr double whell_seperation = 0.26;
        constexpr double whell_diameter = 0.072;
        constexpr double max_linear_vel = 0.2;
        constexpr double min_linear_vel = -1;
        constexpr double max_angular_vel = 0.5;
        constexpr double min_angular_vel = -1;
        constexpr double wp_reach_threshold = 0.2;
        constexpr double max_orientation_diff = 0.2;

        constexpr double robot_radius = 0.3;

        const PID_gains linear_pid(1,0,0);
        const PID_gains angular_pid(1,0,0);
    }
}
#endif //TURTLEBOT_CONSTANTS

