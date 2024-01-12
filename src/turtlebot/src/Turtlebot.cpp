/*
Date: 19.12.2023
Developed by: Ahmet Furkan Akıncı
Project: EE 451 - Project 4
Summary: Includes source codes for a library to control turtlebot
*/

#include <turtlebot/Turtlebot.h>
#include <turtlebot/constants.h>
#include <turtlebot/to_euler.h>
#include <ros/ros.h>
#include <math.h>

namespace Turtlebot {

    Turtlebot::Turtlebot(ros::NodeHandle& nh) : nh(nh) {
        command_velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }

    bool Turtlebot::follow_plan(std::vector<Vector2D> plan, int& current_wp) {
        if (current_wp < plan.size())
        {   
            Vector2D current_pose = Vector2D(0, 0);
            geometry_msgs::Twist command_vel;

            command_vel.linear.x = get_linear_vel(current_pose, plan[current_wp], plan.back(), constants::linear_pid);
            command_vel.angular.z = get_angular_vel(current_pose, plan[current_wp], constants::angular_pid);
            command_velocity_publisher.publish(command_vel);
            
            if (plan[current_wp].substract(current_pose).getLength() < constants::wp_reach_threshold) {
                std::cout << "WAYPOINT " << current_wp << " REACHED" << std::endl;
                current_wp++;
            }
            return false;
        } else {
            return true;
        }
    }

    double Turtlebot::get_linear_vel(Vector2D current_pose, Vector2D current_wp, Vector2D goal, PID_gains pid) {
        Vector2D diff_vec = current_wp.substract(current_pose);
        double dest_orientation = atan2(diff_vec.y, diff_vec.x);
        // double current_orientation_diff = abs(dest_orientation - robot_orientation.z);
        double current_orientation_diff = abs(dest_orientation);

        if (current_orientation_diff > constants::max_orientation_diff) {
            return 0;
        }

        double calculated = -current_pose.substract(goal).x * pid.P;
        
        if (calculated < constants::max_linear_vel) {
            return calculated;
        } else {
            return constants::max_linear_vel;
        }
    }

    double Turtlebot::get_angular_vel(Vector2D current_pose, Vector2D current_wp, PID_gains pid) {
        Vector2D diff_vec = current_wp.substract(current_pose);
        double dest_orientation = atan2(diff_vec.y, diff_vec.x);
        // double current_orientation = robot_orientation.z;
        double current_orientation = 0;
        double calculated = normalizeAngle(dest_orientation - current_orientation) * pid.P;

        if (std::fabs(calculated) < constants::max_angular_vel) {
            return -calculated;
        } else {
            return constants::max_angular_vel * copysign(1.0, -calculated);
        }
    }

    double Turtlebot::normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }
}

