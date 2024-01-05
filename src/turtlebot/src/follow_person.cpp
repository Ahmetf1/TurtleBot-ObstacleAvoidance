/*
Date: 19.12.2023
Developed by: Ahmet Furkan Akıncı
Project: EE 451 - Project 4
Summary: Do a plan with APF and move the robot accordingly
*/

#include <turtlebot/Vector2D.h>
#include <turtlebot/Turtlebot.h>
#include <turtlebot/constants.h>
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "run_plan");
    ros::NodeHandle nh;
    
    
    Turtlebot::Turtlebot turtlebot_controller(nh);
    ros::Rate control_rate(Turtlebot::constants::control_rate);
    ros::Duration(1).sleep();
    ros::spinOnce();
    
    int current_wp = 0;
    bool reached = false;

    std::vector<Vector2D> plan;
    plan.emplace_back(5,0);

    while(true) {
        while(!reached) {
            reached = turtlebot_controller.follow_plan(plan, current_wp);
            ros::spinOnce();
            control_rate.sleep();
        }
        plan.clear();
        current_wp = 0;
        plan.emplace_back(5,0);
    }


    turtlebot_controller.command_velocity_publisher.publish(geometry_msgs::Twist());
}
