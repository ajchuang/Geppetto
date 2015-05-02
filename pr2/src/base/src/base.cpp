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
/* Macros                                                                    */
/*****************************************************************************/
#define M_LINEAR_SPEED  0.25
#define M_ANGULAR_SPEED 0.25

/*****************************************************************************/
/* Global Variables                                                          */
/*****************************************************************************/
ros::NodeHandle *gp_nh;
ros::Publisher *gp_cmd_vel_pub;

/*****************************************************************************/
/* functions                                                                 */
/*****************************************************************************/
void vcmd_action (const std_msgs::String::ConstPtr& msg){
    
    const char* s = msg->data.c_str();
    ROS_INFO ("vcmd: %s",s);
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0.0;

    if (strncmp (s, "go", 2) == 0) {
        base_cmd.linear.x = M_LINEAR_SPEED;
    } else if (strncmp (s, "back", 4) == 0) {
        base_cmd.linear.x = (-1) * M_LIEAR_SPEED;
    } else if (strncmp (s, "stop", 4) == 0) {
        base_cmd.linear.x = 0.0;
    } else if (strncmp (s, "left", 4) == 0) {
        base_cmd.angular.z = M_ANGULAR_SPEED;
    } else if (strncmp (s, "right", 5) == 0) {
        base_cmd.angular.z = (-1) * M_ANGULAR_SPEED;
    } else {
        ROS_INFO ("[vcmd] Sorry I dont understand");
        return;
    }

    /* send the command to the base */
    gp_cmd_vel_pub->publish (base_cmd);
}

int main (int argc, char** argv) {
    ros::init (argc, argv, "base_ctrl");
    gp_nh = new ros::NodeHandle ();  

    /* become a publisher */
    gp_cmd_vel_pub = &(gp_nh->advertise<geometry_msgs::Twist>("/base_controller/command", 1));
    
    /* register as a subscriber */
    ros::Subscriber sub = n->subscribe ("vcmd", 10, vcmd_action); 
    ros::spin();
}
