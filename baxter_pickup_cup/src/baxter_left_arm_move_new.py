#!/usr/bin/env python
"""
Random select a cup. Approach to it. grab the cup. lift it a liite bit and then put it back
press ctrl-c to break the loop
input topic:
/cup_baxter_coordinates
    cup center pose in Baxter coordinate
    data structure: ros custom point array message

"""
import sys
import tf
import math
import rospy
import time
import struct
import numpy as np
import baxter_interface
import baxter_pykdl
import baxter_external_devices
import argparse

from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Header
from baxter_kdl.kdl_parser import kdl_tree_from_urdf_model
from urdf_parser_py.urdf import URDF
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import (Point,PoseStamped,Pose,Quaternion)
from geometry_msgs.msg import Point
from position_calibration.msg import pointarray

from pynput.keyboard import Key, Controller
from random import randint

cupCoordinates = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
index = 0

# Robot arm move method. Grab from Baxter offical website
def IKMove(tranx, trany, tranz, rotax, rotay, rotaz, rotaw):
        right_arm = baxter_interface.Limb('left')
        ns = "ExternalTools/" + 'left' + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            'left': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=tranx,
                        y=trany,
                        z=tranz,
                    ),
                    orientation=Quaternion(
                        x=rotax,
                        y=rotay,
                        z=rotaz,
                        w=rotaw,
                    ),
                ),
            ),
        }
 
        ikreq.pose_stamp.append(poses['left'])
        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return 1
 
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }.get(resp_seeds[0], 'None')
            print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                (seed_str,))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print "\nIK Joint Solution:\n", limb_joints
            print "------------------"
            print "Response Message:\n", resp
            joint_angles = {'left_s0': limb_joints['left_s0'], 'left_s1': limb_joints['left_s1'], 'left_w0': limb_joints['left_w0'], 'left_w1': limb_joints['left_w1'], 'left_w2': limb_joints['left_w2'], 'left_e0': limb_joints['left_e0'], 'left_e1': limb_joints['left_e1']}
            print('Start moving')
            right_arm.move_to_joint_positions(joint_angles)
            print('finished')
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
 
def callback_processed(data):
    global cupCoordinates
    global index
    index = data.index
    for i in range(0, index):
        point = Point()
        point = data.points[i]
        cupCoordinates[0][i] = point.x
        cupCoordinates[1][i] = point.y
        cupCoordinates[2][i] = point.z

def motor_command(a):
    pub=rospy.Publisher('/tilt_controller/command',Float64,queue_size=1)
    msg=Float64(a)
    pub.publish(msg)

def move_strategy(cupCoordinates, index):
    rotax = 0.221644370583
    rotay = 0.973643840426
    rotaz = 0.0533532545114
    rotaw = -0.0066988971153
    try:
        while 1:
            
            random_choice = randint(0, index-1)
            print(random_choice)
            print(cupCoordinates)
            random_choice_translation = np.array([[cupCoordinates[0][random_choice]],
                                                [cupCoordinates[1][random_choice]],
                                                [cupCoordinates[2][random_choice]]])
            if any(np.isnan(random_choice_translation)) == False:
                cupCoordinates_copy = random_choice_translation
                break
    except KeyboardInterrupt:
        print('interrupted!')
    print('target translation: ', cupCoordinates_copy)
    raw_input("Find a valid target. Press Enter to continue...")
    IKMove(cupCoordinates_copy[0], cupCoordinates_copy[1]-0.04, cupCoordinates_copy[2]+0.19, rotax, rotay, rotaz, rotaw)
    raw_input("Check motor. Press Enter to continue...")
    motor_command(-36.5)
    raw_input("wait for grab a cup. Press Enter to continue...")
    IKMove(cupCoordinates_copy[0], cupCoordinates_copy[1]-0.04, cupCoordinates_copy[2]+0.25, rotax, rotay, rotaz, rotaw)
    raw_input("Check if found the ball. Press Enter to continue...")
    IKMove(cupCoordinates_copy[0], cupCoordinates_copy[1]-0.04, cupCoordinates_copy[2]+0.19, rotax, rotay, rotaz, rotaw)
    motor_command(-33)
    IKMove(0.7, 0.5, 0.35, rotax, rotay, rotaz, rotaw)

    
def main(args):
    rospy.init_node('Motion', anonymous=True)
    rospy.Subscriber("/cup_baxter_coordinates", pointarray, callback_processed)
    # move left arm to a preparetion pose first to test the motor 
    IKMove(0.7, 0.5, 0.35, 0.221644370583, 0.973643840426, 0.0533532545114, -0.0066988971153)
    print("Preparation test the hand")
    motor_command(-33)
    raw_input("Begin. Press Enter to continue...")
    while 1:
        raw_input("Start a try. Press Enter to continue...")
        move_strategy(cupCoordinates, index)


if __name__ == '__main__':
    main(sys.argv)