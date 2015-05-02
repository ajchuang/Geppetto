#include <ros/ros.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <iostream>
#include <cstdlib>
#include <sensor_msgs/JointState.h>
#include <string>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

class Gripper{
private:
  GripperClient* gripper_client_;  
  

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    gripper_client_ = new GripperClient("r_gripper_controller/gripper_action", true);
    //wait for the gripper action server to come up 
    while(!gripper_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_controller/gripper_action action server to come up");
    }
  }

  ~Gripper(){
    delete gripper_client_;
 
  }   

  //Open the gripper
  void open(){
    pr2_controllers_msgs::Pr2GripperCommandGoal open;
    //position ranges from 0 to 0.09    
    open.command.position = 0.09;       
    //force is in N
    open.command.max_effort = -1;  // Do not limit effort (negative)
    
    ROS_INFO("Sending open goal");
    gripper_client_->sendGoal(open);
    gripper_client_->waitForResult();
    ros::Duration(20).sleep();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper opened!");
    else
      ROS_INFO("The gripper failed to open.");
  }

  //Close the gripper
  void close(){
    pr2_controllers_msgs::Pr2GripperCommandGoal squeeze;
    squeeze.command.position = 0.00;
    squeeze.command.max_effort = 30.0;  // Close gently
    
    ROS_INFO("Sending squeeze goal");
    gripper_client_->sendGoal(squeeze);
    gripper_client_->waitForResult();
    ros::Duration(20).sleep();
    if(gripper_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("The gripper closed!");
    else
      ROS_INFO("The gripper failed to close.");
  }
};

/*****************************************************************************/
/* Global variable                                                           */
/*****************************************************************************/
Gripper *gripper = NULL;
TrajClient* traj_client_ = NULL;


//right forearm roll
void startTrajectory (pr2_controllers_msgs::JointTrajectoryGoal goal){
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.3);
    traj_client_->sendGoal (goal);
}

pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory (double* data){    
    //our goal variable
    pr2_controllers_msgs::JointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.points.resize(2);

    //First point
    int ind = 0;
    // position
    goal.trajectory.points[ind].positions.resize(1);    
    goal.trajectory.points[ind].positions[0] = 0.0;
    // velocity
    goal.trajectory.points[ind].velocities.resize(1);
    goal.trajectory.points[ind].velocities[0] = 0.0;

    goal.trajectory.points[ind].time_from_start = ros::Duration(.3);

    //Second point
    ind += 1;
    // position
    goal.trajectory.points[ind].positions.resize(1);
    goal.trajectory.points[ind].positions[0] = *data;
    // velocity
    goal.trajectory.points[ind].velocities.resize(1);
    goal.trajectory.points[ind].velocities[0] = *data;
    
    goal.trajectory.points[ind].time_from_start = ros::Duration(.8);
    
    return goal;
}



/*****************************************************************************/
/* Global function                                                           */
/*****************************************************************************/
void action (const std_msgs::String::ConstPtr& msg){
    char state[2];    
    const char* s = msg->data.c_str();
    ROS_INFO ("%s",s);
    char data[256];
    strcpy (data, s);

    char* x = strtok(data, " ");
    char* y = strtok(data, " \0");   
    
    ROS_INFO ("Starting to move");
    double roll = atof (y);
    startTrajectory (armExtensionTrajectory (&roll));

    // check if the move is completed.
    ROS_INFO ("Waiting for ARM move completion");
	
    while(!traj_client_->getState().isDone() && ros::ok()) {
        usleep(50000);
    }

    ROS_INFO ("ARM move completed");


    switch(x[0]) {
    case 'o':
        gripper->open ();
    break;

    case 'c':
        gripper->close ();
    break;

    default:
        ROS_INFO ("Incorrect command: %s", s);
    break;
    }
}

int main (int argc, char** argv) {
    ros::NodeHandle *n = NULL;  
    ros::init(argc, argv, "simple_gripper");
    n = new ros::NodeHandle ();   
    gripper = new Gripper ();     
    traj_client_ = new TrajClient ("r_arm_controller/joint_trajectory_action", true);  
    ros::Subscriber sub = n->subscribe ("myo", 1000, action); 
    ros::spin();
}
