/*
Date: 19.12.2023
Developed by: Ahmet Furkan Akıncı
Project: EE 451 - Project 4
Summary: Includes a library to control turtlebot
*/


#ifndef TURTLEOT_TURTLEBOT_H
#define TURTLEOT_TURTLEBOT_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#ifdef SIMULATION
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#endif


#include <turtlebot/Vector2D.h>

namespace Turtlebot {

    struct PID_gains;

    class Turtlebot {
    public:
        /*
        Date: 19.12.2023
        Developed by: Ahmet Furkan Akıncı
        Summary: Constructor for the Turtlebot class. Initializes the Turtlebot with a ROS node handle.
        Input: ros::NodeHandle& nh - Reference to the ROS node handle
        Output: N/A
        Additional info: Sets up publishers and subscribers for the Turtlebot
        */
        Turtlebot(ros::NodeHandle& nh);


        /*
        Date: 19.12.2023
        Developed by: Ahmet Furkan Akıncı
        Summary: Commands the robot to follow a specified plan.
        Input: std::vector<Vector2D> plan, int& current_wp
        Output: bool - Success or failure of following the plan
        Additional info: The plan consists of waypoints the robot should follow
        */
        bool follow_plan(std::vector<Vector2D> plan, int& current_wp);
        
        /*
        Date: 20.12.2023
        Developed by: Ahmet Furkan Akıncı
        Summary: Normalizes angle between -pi and pi
        Input: double angle
        Output: double - Normalized angle
        */
        double normalizeAngle(double angle);

        ros::Publisher command_velocity_publisher;

    private:
        /*
        Date: 19.12.2023
        Developed by: Ahmet Furkan Akıncı
        Summary: Calculates the linear velocity for the robot to reach a goal position.
        Input: Vector2D current_pose, Vector2D goal, PID_gains pid
        Output: double - Calculated linear velocity
        Additional info: Uses PID control for smooth and accurate motion
        */
        double get_linear_vel(Vector2D current_pose, Vector2D current_wp, Vector2D goal, PID_gains pid);

        /*
        Date: 19.12.2023
        Developed by: Ahmet Furkan Akıncı
        Summary: Calculates the angular velocity for the robot to reach a waypoint.
        Input: Vector2D current_pose, Vector2D current_wp, PID_gains pid
        Output: double - Calculated angular velocity
        Additional info: Uses PID control for accurate orientation adjustment
        */
        double get_angular_vel(Vector2D current_pose, Vector2D current_wp, PID_gains pid);

        ros::NodeHandle nh;
    };

    struct PID_gains {
        PID_gains(double P, double I, double D): P(P), I(I), D(D) {};
        double P;
        double I;
        double D;
    };
}
#endif //TURTLEOT_TURTLEBOT_H

