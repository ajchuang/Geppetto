import rospy, std_msgs.msg, time
from std_msgs.msg import String
from trajectory_msgs.msg import (JointTrajectory, JointTrajectoryPoint)
from copy import copy

if __name__ == "__main__":
    
    pub = rospy.Publisher ("/head_traj_controller/command", JointTrajectory, queue_size=10)
    rospy.init_node ('HeadTajectoryPublisher', anonymous=True)
    rate = rospy.Rate (10) # 10hz
    
    print "node initialized"
    while True:
	    JTpoint = JointTrajectoryPoint()
	    JTpoint.positions = [1, 1]
	    JTpoint.velocities = [0,0]
	    JTpoint.accelerations = [0,0]
	    #JTpoint.time_from_start = [0,0]
	    JTmsg = JointTrajectory()
	    JTmsg.joint_names = ["head_pan_joint", "head_tilt_joint"]
	    JTmsg.points = []
	    JTmsg.points.append(copy(JTpoint))
	    print JTmsg
	    pub.publish(JTmsg)
	    time.sleep(1)
