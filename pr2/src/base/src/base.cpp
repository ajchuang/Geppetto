#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <iostream>
#include <cstdlib>
#include <sensor_msgs/JointState.h>
#include <string>

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
ros::NodeHandle *gp_nh;
ros::Publisher *gp_cmd_vel_pub;

void vcmd_action (const std_msgs::String::ConstPtr& msg){
    
    const char* s = msg->data.c_str();
    ROS_INFO ("vcmd: %s",s);
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0.0;

    if (strcmp (s, "go") == 0) {
        base_cmd.linear.x = 0.25;
    } else if (strcmp (s, "stop") == 0) {
        base_cmd.linear.x = 0.0;
    }

    gp_cmd_vel_pub->publish(base_cmd);
}

int main (int argc, char** argv) {
    ros::NodeHandle *n = NULL;  
    ros::init(argc, argv, "base_ctrl");
    gp_nh = new ros::NodeHandle ();  
    gp_cmd_vel_pub = &(gp_nh->advertise<geometry_msgs::Twist>("/base_controller/command", 1));
    ros::Subscriber sub = n->subscribe ("vcmd", 10, vcmd_action); 
    ros::spin();
}
