/*
Date: 13.01.2023
Developed by: Ahmet Furkan Akıncı
Project: EE 451 - Final Project
Summary: Follow an object 
*/

#include <turtlebot/Vector2D.h>
#include <turtlebot/Turtlebot.h>
#include <turtlebot/constants.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>

geometry_msgs::Point target_position;

/*
Date: 12.01.2023
Developed by: Ahmet Furkan Akıncı
Summary: Callback function for updating target position from ROS message.
Input: ROS message containing the target position (geometry_msgs::Point).
Output: The global variable 'target_position' is updated with the received position.
*/
void target_position_cb(const geometry_msgs::Point::ConstPtr& msg);

/*
Date: 12.01.2023
Developed by: Ahmet Furkan Akıncı
Summary: Calculates the waypoint position based on the target position and a specified follow distance.
Input: 
    - target_position: The current target position as a 2D vector.
    - follow_distance: The distance to maintain from the target position.
Output: Returns a Vector2D object representing the calculated waypoint position.
*/
Vector2D calculate_wp_position(Vector2D target_position, double follow_distance);

int main(int argc, char** argv) {
    ros::init(argc, argv, "run_plan");
    ros::NodeHandle nh;
    
    ros::Subscriber target_position_sub = nh.subscribe<geometry_msgs::Point>("/detection_result/blue", 10, target_position_cb);
    Turtlebot::Turtlebot turtlebot_controller(nh);
    ros::Rate control_rate(Turtlebot::constants::control_rate);
    ros::Duration(1).sleep();
    ros::spinOnce();
    
    int current_wp = 0;
    bool reached = false;

    std::vector<Vector2D> plan;
    plan.emplace_back(target_position.x, target_position.y);

    while(true) {
        reached = turtlebot_controller.follow_plan(plan, current_wp);
        ros::spinOnce();
        control_rate.sleep();
        plan.clear();
        current_wp = 0;
        Vector2D wp_position = calculate_wp_position(Vector2D(target_position.x, target_position.y), 0.5);
        plan.emplace_back(wp_position.x, wp_position.y);
    }


    turtlebot_controller.command_velocity_publisher.publish(geometry_msgs::Twist());
}

void target_position_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    target_position = *msg;
}

Vector2D calculate_wp_position(Vector2D target_position, double follow_distance) {
    double dest_orientation = atan2(target_position.y, target_position.x);
    double wp_x = target_position.x - follow_distance * cos(dest_orientation);
    double wp_y = target_position.y - follow_distance * sin(dest_orientation);
    return Vector2D(wp_x, wp_y);
}