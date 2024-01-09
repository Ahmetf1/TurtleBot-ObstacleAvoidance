#include <turtlebot/path_planner.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>
#include <turtlebot/Vector2D.h>
#include <turtlebot/Turtlebot.h>
#include <turtlebot/constants.h>

geometry_msgs::Point targetPose;
void targetPoseCallback(const geometry_msgs::Point::ConstPtr& msg) {
    targetPose.x = msg->x;
    targetPose.y = msg->y;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_BFS");
    ros::NodeHandle nh;


    // Create a subscriber object
    ros::Subscriber target_position_sub = nh.subscribe("target_pose", 10, targetPoseCallback);

    ros::Rate control_rate(20);
    ros::Duration(1).sleep();
    ros::spinOnce();

    double resolution = 0.2;
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

    while(ros::ok()){
        if(counter == 20){
            std::vector<std::pair<double, double>> points;
            points.push_back({targetPose.x, targetPose.y});
            gridGen.adjustGridSizeForCoordinates(points);
            gridGen.setDestination(targetPose.x, targetPose.y);

            gridGen.printGrid();
            ROS_INFO("Target: %f, %f", targetPose.x, targetPose.y);
            std::vector<std::pair<double, double>> path = pathGen.findPath_BFS();
            pathGen.printPath(path);
            for(auto p : path){
                plan.push_back({p.first, p.second});
            }
            ROS_INFO("test4");
            counter = 0;
        }
        while(!reached) {
            reached = turtlebot_controller.follow_plan(plan, current_wp);
            ros::spinOnce();
            control_rate.sleep();
            //turtlebot_controller.command_velocity_publisher.publish(geometry_msgs::Twist());
        }
        
        plan.clear();
        current_wp = 0;
        
        turtlebot_controller.command_velocity_publisher.publish(geometry_msgs::Twist());
        counter++;
        ros::spinOnce();
        control_rate.sleep();

    }




}