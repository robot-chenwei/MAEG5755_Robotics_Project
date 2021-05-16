#!/usr/bin/env python3

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
"""
import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from moveit_task_constructor_dexnet.srv import GQCNNGrasp, GQCNNGraspResponse, GQCNNGraspRequest
from baxter_pykdl import baxter_kinematics
import numpy

class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        self._kin = baxter_kinematics('left')

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        joint_pos = {'left_w0': 0, 'left_w1': 0, 'left_w2': 0, 'left_e0': 0, 'left_e1': 0, 'left_s0': 0, 'left_s1': 0}
        xpos = [pose.position.x, pose.position.y, pose.position.z]
        xori = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        jpos = self._kin.inverse_kinematics(xpos, xori)  
        if jpos:
            joint_pos['left_s0'] = jpos.item(0)[0]
            joint_pos['left_s1'] = jpos.item(0)[1]
            joint_pos['left_e0'] = jpos.item(0)[2] 
            joint_pos['left_e1'] = jpos.item(0)[3]
            joint_pos['left_w0'] = jpos.item(0)[4]
            joint_pos['left_w1'] = jpos.item(0)[5]
            joint_pos['left_w2'] = jpos.item(0)[6]
        return joint_pos

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def main():
    rospy.init_node("baxter_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    pnp = PickAndPlace(limb, hover_distance)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                             x=-0.0249590815779,
                             y=0.999649402929,
                             z=0.00737916180073,
                             w=0.00486450832011)
                             
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    print('calculate pose')
    try: 
        rospy.wait_for_service('gqcnn_grasp')
        gqcnn_grasp = rospy.ServiceProxy('gqcnn_grasp', GQCNNGrasp)
        req = GQCNNGraspRequest()
        req.color_img_file_path = '/home/trs/ros_ws/src/MAEG5755_Robotics_Project/dexnet/deep_grasp_demo/moveit_task_constructor_dexnet/data/images/rgb_cylinder_overhead.png'
        req.depth_img_file_path = '/home/trs/ros_ws/src/MAEG5755_Robotics_Project/dexnet/deep_grasp_demo/moveit_task_constructor_dexnet/data/images/depth_cylinder_overhead.png'
        res = gqcnn_grasp(req)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
    res_size = 0
    if res:
        res_size = len(res.q_values)
    if res_size:
        q_values = res.q_values[0]
        grasps = res.grasps[0]
        for i in range(1, res_size):
    	    if q_values < res.q_values[i]:
    	        q_values = res.q_values[i]
    	        grasps = res.grasps[i]
        print("get pose from gqcnn_grasp with max q_values")
        print(grasps)

    block_poses = list()
    block_poses.append(grasps.pose)
    block_poses.append(Pose(
        position=Point(x=0.75, y=0.0, z=-0.129),
        orientation=overhead_orientation))
        
    # Move to the desired starting angles
    pnp.move_to_start(starting_joint_angles)
    idx = 0
    while not rospy.is_shutdown():
        print("\nPicking...")
        pnp.pick(block_poses[idx])
        print("\nPlacing...")
        idx = (idx+1) % len(block_poses)
        pnp.place(block_poses[idx])
    return 0

if __name__ == '__main__':
    sys.exit(main())
