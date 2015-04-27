import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import (JointTrajectory, JointTrajectoryPoint)
from sensor_msgs.msg import Image



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "positions[0] %s", str(data.positions[0]))
    rospy.loginfo(rospy.get_caller_id() + "positions[0] %s", str(data.positions[1]))


if __name__ == "__main__":
    rospy.init_node('Subscriber', anonymous=True)
    rospy.Subscriber("/head_traj_controller/command", JointTrajectory, callback)
    rospy.spin()
