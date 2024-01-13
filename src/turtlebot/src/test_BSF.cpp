#include <turtlebot/path_planner.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <turtlebot/Vector2D.h>
#include <turtlebot/Turtlebot.h>
#include <turtlebot/constants.h>

geometry_msgs::Point target_position;
geometry_msgs::Point lastPose;
bool isFirstCallback = true;
ros::Time lastCallbackTime;

bool check_close(Vector2D target_pos, double follow_distance){
    double dist = sqrt(pow(target_pos.x, 2) + pow(target_pos.y, 2));
    return (dist > follow_distance);
}
Vector2D calculate_wp_position(Vector2D target_position, double follow_distance) {
    double dest_orientation = atan2(target_position.y, target_position.x);
    double wp_x = target_position.x - follow_distance * cos(dest_orientation);
    double wp_y = target_position.y - follow_distance * sin(dest_orientation);
    return Vector2D(wp_x, wp_y);
}

void targetPoseCallback(const geometry_msgs::Point::ConstPtr& msg) {
    if (std::isnan(msg->x) || std::isnan(msg->y)) {
        ROS_WARN("Received NaN data. Ignoring this update.");
        target_position.x = 0;
        target_position.y = 0;
        // Optionally, also reset lastPose to 0
        lastPose.x = 0.0;
        lastPose.y = 0.0;
    }
    // Check if this is the first callback
    if (isFirstCallback) {
        isFirstCallback = false;
    } else {
        // Check if the data is the same as last time
        if (msg->x == lastPose.x && msg->y == lastPose.y) {
            ROS_INFO("Received identical position data.");
            // Handle identical data case here
        }
    }

    // Update the target pose and last pose
    target_position.x = msg->x;
    target_position.y = msg->y;
    lastPose = target_position;

    // Update the last callback time
    lastCallbackTime = ros::Time::now();
}
void checkForNoData(const ros::TimerEvent&) {
    ros::Duration noDataDuration = ros::Time::now() - lastCallbackTime;
    // Check if the duration since the last callback is over a threshold
    if (noDataDuration.toSec() > 0.2) {
        ROS_WARN("No new data received for over 0.2 seconds.");
        // Handle no data case here
        target_position.x = lastPose.x;
        target_position.y = lastPose.y;
        // Optionally, also reset lastPose to 0
        lastPose.x = 0.0;
        lastPose.y = 0.0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_BFS");
    ros::NodeHandle nh;


    // Create a subscriber object
    ros::Subscriber target_position_sub = nh.subscribe<geometry_msgs::Point>("/detection_result/blue", 10, targetPoseCallback);
    ros::Timer noDataTimer = nh.createTimer(ros::Duration(0.1), checkForNoData);

    ros::Rate control_rate(40);
    ros::Duration(1).sleep();
    ros::spinOnce();

    double resolution = 0.3;
    GridGenerator gridGen(resolution);

    PathFinder pathGen(gridGen);

    int counter = 0;
    // Test the path generation
    // while(ros::ok()){
    //     if(counter == 20){
    //         std::vector<std::pair<double, double>> points;
    //         points.push_back({targetPose.x, targetPose.y});
    //         gridGen.adjustGridSizeForCoordinates(points);
    //         gridGen.setDestination(targetPose.x, targetPose.y);

    //         gridGen.printGrid();
    //         ROS_INFO("Target: %f, %f", targetPose.x, targetPose.y);
    //         std::vector<std::pair<double, double>> path = pathGen.findPath_BFS();
    //         pathGen.printPath(path);
    //         ROS_INFO("test4");
    //         counter = 0;
    //     }
    //     counter++;
    //     ros::spinOnce();
    //     control_rate.sleep();

    // }
    Turtlebot::Turtlebot turtlebot_controller(nh);

    ros::Duration(1).sleep();
    ros::spinOnce();
    
    int current_wp = 0;
    bool reached = false;

    std::vector<Vector2D> plan;
    // // //Test for the turtlebot controller
    // plan.emplace_back(targetPose.x, targetPose.y);
    // while (ros::ok())
    // {
        
    //     reached = turtlebot_controller.follow_plan(plan, current_wp);
    //     ros::spinOnce();
    //     control_rate.sleep();
        
    //     plan.clear();
    //     current_wp = 0;
    //     plan.emplace_back(targetPose.x, targetPose.y);
    //     std::cout<< "Target : " << targetPose.x << " " << targetPose.y << "\n";
    // }
    

    while(ros::ok()){
        if(counter == 5){
            std::vector<std::pair<double, double>> points;
            points.push_back({target_position.x, target_position.y});
            gridGen.adjustGridSizeForCoordinates(points);
            gridGen.setDestination(target_position.x, target_position.y);

            gridGen.printGrid();
            ROS_INFO("Target: %f, %f", target_position.x, target_position.y);
            std::vector<std::pair<double, double>> path = pathGen.findPath_BFS();
            pathGen.printPath(path);
            std::pair<double, double> maxDist = pathGen.findFurthestCollinearPoint(path);
            ROS_INFO("DIST: %f %f", maxDist.first, maxDist.second); 
            Vector2D ttt;
            ttt.x = maxDist.first;
            ttt.y = maxDist.second;
            Vector2D fff;
            fff.x = target_position.x;
            fff.y = target_position.y;
            // bool flag = check_close(fff, 0.2);
            // std::cout<<"FLAG: " << flag << "\n";


            Vector2D xxx = calculate_wp_position(ttt, 0.2);
            plan.emplace_back(xxx.x, xxx.y);
           
            
            reached = turtlebot_controller.follow_plan(plan, current_wp);
            ros::spinOnce();
            control_rate.sleep();
            

            plan.clear();
            current_wp = 0;
            counter = 0;
        }

        counter++;
        ros::spinOnce();
        control_rate.sleep();

    }



}