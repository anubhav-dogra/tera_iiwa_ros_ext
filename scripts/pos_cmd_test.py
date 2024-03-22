#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import numpy as np

joints=[]
def callback(data):
    global joints
    joints = data.position

rospy.init_node('test_node')
pub=rospy.Publisher("/iiwa/PositionController/command", Float64MultiArray, queue_size=10)
rospy.Subscriber("/iiwa/joint_states",JointState,callback)

lower_limits = -np.array([170, 120, 170, 120, 170, 120 ,175])
lower_limits = lower_limits*np.pi/180.
upper_limits = np.array([170, 120, 170, 120, 170, 120 ,175])
upper_limits = upper_limits*np.pi/180.

msg=Float64MultiArray()

if not joints:
    rospy.sleep(0.5)
msg.data = [joints[0], joints[1], joints[2], joints[3], joints[4], joints[5],joints[6]]    
# msg.data = [-0.77, -1.2048, -1.694, -1.7794, -1.1734, 1.416, -1.57]

rate = rospy.Rate(20)
while not rospy.is_shutdown():
    
    time = rospy.get_time()
    angle = math.sin(time)
    angle = max(lower_limits[6],min(angle,upper_limits[6]))
    msg.data[6] = msg.data[6]+0.01*angle
    pub.publish(msg)
    print(msg)

    rate.sleep()