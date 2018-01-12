#!/usr/bin/env python
# license removed for brevity
import rospy
from  brics_actuator.msg import JointTorques, JointValue
from  sensor_msgs.msg import JointState


angle_des = 2.5

def callback(data):
    angle_cur = data.position[4]
    u = 0.5*(angle_des - angle_cur)
    msg = JointTorques()
    msg.torques.append(JointValue())
    msg.torques[0].joint_uri = "arm_joint_5"
    msg.torques[0].unit = "m^2 kg s^-2 rad^-1"
    msg.torques[0].value = u
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pcontroller_node')
    pub = rospy.Publisher('/arm_1/arm_controller/torques_command', JointTorques, queue_size=10)
    sub = rospy.Subscriber("/joint_states", JointState, callback)
    rospy.spin()
