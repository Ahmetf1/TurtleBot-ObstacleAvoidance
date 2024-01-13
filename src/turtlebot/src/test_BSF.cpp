/*
Date:13/01/2024
Developed by: Musa Almaz, Ahmet Furkan Akıncı
Project: Final Project
Summary: This file is the main siulation file for the obstacle avoidance ath target following.
It uses PID when there is no obstacle and uses BFS path finding to avoid obstacle.
*/ 
#include <turtlebot/path_planner.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <turtlebot/Vector2D.h>
#include <turtlebot/Turtlebot.h>
#include <turtlebot/constants.h>

// Global variables to store the target position and the last known pose
geometry_msgs::Point target_position;
geometry_msgs::Point last_pose;

// Flag to check if it's the first callback for target pose
bool isFirstCallback = true;

/*
Date: 13.01.2024
Developed by: Musa Almaz, Ahmet Furkan Akıncı
Summary: Calculates the waypoint position based on the target position and a specified follow distance.
Input: Target position as a Vector2D and follow distance as a double
Output: Waypoint position as a Vector2D
Additional info: The function calculates the position behind the target at a specified distance.
*/
Vector2D calculate_wp_position(Vector2D target_position, double follow_distance) {
    double dest_orientation = atan2(target_position.y, target_position.x);
    double wp_x = target_position.x - follow_distance * cos(dest_orientation);
    double wp_y = target_position.y - follow_distance * sin(dest_orientation);
    return Vector2D(wp_x, wp_y);
}

/*
Date: 13.01.2024
Developed by: Musa Almaz, Ahmet Furkan Akıncı
Summary: Callback function for target pose updates.
Input: Constant pointer to a geometry_msgs::Point message
Output: No output (updates global variables)
Additional info: Updates the global target_position and last_pose. Handles the first callback case and invalid (NaN) positions.
*/
void targetPoseCallback(const geometry_msgs::Point::ConstPtr& msg) {
    if (isFirstCallback) {
        last_pose = geometry_msgs::Point();
        isFirstCallback = false;
    } 
    if (std::isnan(msg->x) || std::isnan(msg->y)) {
        target_position = last_pose;
    }
    else {
        target_position.x = msg->x;
        target_position.y = msg->y;
        last_pose = target_position;
    }
}

// Global variable to store the obstacle detection status and position
bool obstacle_detected = false;
geometry_msgs::Point obstacle_position;

/*
Date: 13.01.2024
Developed by: Musa Almaz, Ahmet Furkan Akıncı
Summary: Callback function for obstacle position updates.
Input: Constant pointer to a geometry_msgs::Point message
Output: No output (updates global variables)
Additional info: Sets the obstacle_detected flag and updates the obstacle_position based on the received message.
*/
void obstacleCallback(const geometry_msgs::Point::ConstPtr& msg) {
    // Check if the message signifies an obstacle (for example, non-zero coordinates)
    if (!(std::isnan(msg->x) || std::isnan(msg->y))) {
        obstacle_detected = true;
        obstacle_position = *msg;  // Store the obstacle's position
    } else {
        obstacle_detected = false;
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "test_BFS");
    ros::NodeHandle nh;


    // Create a subscriber object
    ros::Subscriber target_position_sub = nh.subscribe<geometry_msgs::Point>("/detection_result/blue", 10, targetPoseCallback);
    ros::Subscriber obstacle_position_sub = nh.subscribe<geometry_msgs::Point>("/detection_result/orange", 10, obstacleCallback);

    ros::Rate control_rate(40);
    ros::Duration(1).sleep();
    ros::spinOnce();

    double resolution = 0.3;
    double obstacle_radius = 0.4;
    GridGenerator gridGen(resolution);

    PathFinder pathGen(gridGen);

    int counter = 0;

    Turtlebot::Turtlebot turtlebot_controller(nh);

    ros::Duration(1).sleep();
    ros::spinOnce();
    
    int current_wp = 0;
    bool reached = false;

    std::vector<Vector2D> plan;

    while(ros::ok()){
        if(counter == 5){
            counter = 0;
            if(obstacle_detected){
                ROS_INFO("OBSTACLE DETECTED");
                std::vector<std::pair<double, double>> points;
                points.push_back({target_position.x, target_position.y});
                points.push_back({obstacle_position.x, obstacle_position.y});
                gridGen.adjustGridSizeForCoordinates(points);
                gridGen.setObstacle(obstacle_position.x, obstacle_position.y, obstacle_radius);
                gridGen.setDestination(target_position.x, target_position.y);
                gridGen.printGrid();

                std::vector<std::pair<double, double>> path = pathGen.findPath_BFS();
                pathGen.printPath(path);
                std::pair<double, double> maxDist = pathGen.findFurthestCollinearPoint(path);

                Vector2D ttt;
                ttt.x = maxDist.first;
                ttt.y = maxDist.second;

                Vector2D xxx = calculate_wp_position(ttt, 0.2);
                plan.emplace_back(xxx.x, xxx.y);            
                
                reached = turtlebot_controller.follow_plan(plan, current_wp);
                ros::spinOnce();
                control_rate.sleep();               

                plan.clear();
                current_wp = 0;
                
            }else{
                Vector2D wp_position = calculate_wp_position(Vector2D(target_position.x, target_position.y), 0.5);
                plan.emplace_back(wp_position.x, wp_position.y);
                reached = turtlebot_controller.follow_plan(plan, current_wp);
                ros::spinOnce();
                control_rate.sleep();
                plan.clear();
                current_wp = 0;
                wp_position = calculate_wp_position(Vector2D(target_position.x, target_position.y), 0.5);
                plan.emplace_back(wp_position.x, wp_position.y);
            }

        }

        counter++;
        ros::spinOnce();
        control_rate.sleep();

    }

}