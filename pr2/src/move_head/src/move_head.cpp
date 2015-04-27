#include <ros/ros.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

/* Prototype */
void movement (const std_msgs::String::ConstPtr& msg);

/* Global Variable*/

ros::Subscriber sub;

class RobotHead
{
private:
  PointHeadClient* point_head_client_;

public:
  //! Action client initialization 
  RobotHead()
  {
    //Initialize the client for the Action interface to the head controller
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);

    //wait for head controller action server to come up 
    while(!point_head_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
  }

  ~RobotHead()
  {
    delete point_head_client_;
  }

  //! Points the high-def camera frame at a point in a given frame  
  void lookAt(std::string frame_id, double x, double y, double z)
  {
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "high_def_frame";

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1

.0;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there (abort after 2 secs to prevent getting stuck)
    point_head_client_->waitForResult(ros::Duration(2));
  }

  //! Shake the head from left to right n times  
  /* void shakeHead(int n)
  {
    int count = 0;
    while (ros::ok() && ++count <= n )
    {
      //Looks at a point forward (x=5m), slightly left (y=1m), and 1.2m up
      lookAt("base_link", -.181, -.017, -.625);

      //Looks at a point forward (x=5m), slightly right (y=-1m), and 1.2m up
      lookAt("base_link", 1, 1, 1);
    }
  }*/
};
 

RobotHead* head;

void movement (const std_msgs::String::ConstPtr& msg){
    double state[3];
    ROS_INFO ("callback [%s]", msg->data.c_str ());
    const char*s = msg->data.c_str ();
    char data[256];
    strcpy (data, s);    

    char* x = strtok (data, " ");     
    int i = 0;

    while (x != NULL) {    
        state[i] = atof(x);
        i++;
        x = strtok (NULL, " ");   
    }
    
    head->lookAt("base_link",state[0],state[1],state[2]);
}

int main(int argc, char** argv)
{
  ros::NodeHandle *n = NULL;
  ros::init(argc, argv, "robot_driver");
  //init the ROS node
  head = new RobotHead ();
  n = new ros::NodeHandle();
  sub = n->subscribe ("Oculus", 1000, movement);
  ros::spin ();
 
}
