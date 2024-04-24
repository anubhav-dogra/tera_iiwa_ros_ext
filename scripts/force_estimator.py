#!/usr/bin/env python3
import rospy
import rbdyn
import numpy as np

# robot_description = rospy.get_param("/robot_description")
# # print(robot_description)
# rbd = rbdyn.parsers.from_urdf(robot_description)
rbd = rbdyn.parsers.from_urdf_file('/home/terabotics/stuff_ws/src/iiwa_ros/iiwa_description/urdf/iiwa14.urdf.xacro')
mbc = rbdyn.MultiBodyConfig(rbd.mb)
mbg = rbdyn.MultiBodyGraph()
mbc.zero(rbd.mb)
rbdyn.forwardKinematics(rbd.mb, rbd.mbc)
rbdyn.forwardVelocity(rbd.mb, rbd.mbc)
eef_id = rbd.mb.bodyIdByName('iiwa_link_7')
print(eef_id)
jac = rbdyn.Jacobian(rbd.mb,eef_id)


# fd = rbdyn.ForwardDynamics(rbd.mb)
# rbdyn.forwardKinematics(rbd.mb, rbd.mbc)
# rbdyn.forwardVelocity(rbd.mb, rbd.mbc)
# fd.computeH(rbd.mb,rbd.mbc)
# H = fd.H()
# print(H)
