import rospy, std_msgs.msg
from std_msgs.msg import String
from trajectory_msgs.msg import (JointTrajectory, JointTrajectoryPoint)


if __name__ == "__name__":

    pub = rospy.Publisher("/head_traj_controller/command", JointTrajectory)
    rospy.init_node('Head Tajectory Publisher')


    JTpoint = JointTrajectoryPoint()
    JTpoint.velocities = [0,0],
    JTpoint.accelerations = [0,0],
    JTpoint.time_from_start = [0,0]
    JTmsg = JointTrajectory()
    JTmsg.joint_names = ["head_pan_joint", "head_tilt_joint"]
    JTmsg.points = []
    for i in range(0, 100):
        if i % 2:
            JTpoint.positions = [0,1]
        else:
            JTpoint.positions = [1,0]
        JTmsg.points.append(copy(JTpoint))
    pub.Publish(JTmsg)
