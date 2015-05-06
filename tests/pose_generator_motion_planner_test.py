#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.abspath(os.path.dirname(__file__) + '/../src/'))
import rospy
from task_motion_planner.srv import *
from task_motion_planner.msg import *
from pose_generator import *
from std_msgs.msg import *
from moveit_msgs.msg import *
from shape_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from motion_plan_playback import *


def pickupTest(action, motion_server, poseGen):
    state = world_state()
    state.world = world_obj()
    
    surf1 = CollisionObject()
    surf1.header = Header()
    surf1.header.frame_id = '/base'
    surf1.id = 'surf1'
    primitive = SolidPrimitive()
    primitive.type = 1
    primitive.dimensions = [1, 1, 0.05]
    surf1.primitives.append(primitive)
    pose = Pose()
    pose.position.x = 1
    pose.position.y = 0
    pose.position.z = 0
    pose.orientation.w = 1
    surf1.primitive_poses.append(pose)
    state.world.surfaces.append(surf1)
    
    obj1 = CollisionObject()
    obj1.header = Header()
    obj1.header.frame_id = '/base'
    obj1.id = 'obj1'
    primitive = SolidPrimitive()
    primitive.type = 3
    primitive.dimensions = [.25, .02]
    obj1.primitives.append(primitive)
    pose = Pose()
    pose.position.x = 0.8
    pose.position.y = 0.15
    pose.position.z = 0.16
    pose.orientation.w = 1
    obj1.primitive_poses.append(pose)
    state.world.movable_objects.append(obj1)
    
    obj2 = CollisionObject()
    obj2.header = Header()
    obj2.header.frame_id = '/base'
    obj2.id = 'obj2'
    primitive = SolidPrimitive()
    primitive.type = 3
    primitive.dimensions = [.25, .02]
    obj2.primitives.append(primitive)
    pose = Pose()
    pose.position.x = 0.75
    pose.position.y = 0.15
    pose.position.z = 0.16
    pose.orientation.w = 1
    obj2.primitive_poses.append(pose)
    state.world.movable_objects.append(obj2)
    
    obj3 = CollisionObject()
    obj3.header = Header()
    obj3.header.frame_id = '/base'
    obj3.id = 'obj3'
    primitive = SolidPrimitive()
    primitive.type = 3
    primitive.dimensions = [.25, .02]
    obj3.primitives.append(primitive)
    pose = Pose()
    pose.position.x = 0.8
    pose.position.y = 0.1
    pose.position.z = 0.16
    pose.orientation.w = 1
    obj3.primitive_poses.append(pose)
    state.world.movable_objects.append(obj3)
    
    obj4 = CollisionObject()
    obj4.header = Header()
    obj4.header.frame_id = '/base'
    obj4.id = 'obj4'
    primitive = SolidPrimitive()
    primitive.type = 3
    primitive.dimensions = [.25, .02]
    obj4.primitives.append(primitive)
    pose = Pose()
    pose.position.x = 0.8
    pose.position.y = 0.2
    pose.position.z = 0.16
    pose.orientation.w = 1
    obj4.primitive_poses.append(pose)
    state.world.movable_objects.append(obj4)
    
    obj5 = CollisionObject()
    obj5.header = Header()
    obj5.header.frame_id = '/base'
    obj5.id = 'obj5'
    primitive = SolidPrimitive()
    primitive.type = 3
    primitive.dimensions = [.25, .02]
    obj5.primitives.append(primitive)
    pose = Pose()
    pose.position.x = 0.85
    pose.position.y = 0.15
    pose.position.z = 0.16
    pose.orientation.w = 1
    obj5.primitive_poses.append(pose)
    state.world.movable_objects.append(obj5)

    state.robot = robot()
    state.robot.id = 'left_arm'
    state.robot.state = RobotState()
    state.robot.state.joint_state.name = ['head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    state.robot.state.joint_state.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    state.robot.state.joint_state.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    state.robot.state.joint_state.effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    msg = motion_plan_parameters()
    msg.state = state
    msg.action = action
    msg.goals = poseGen.next(action, state)
    
    #goals = [pose_gen()]
    #goals[0].pose.position.x = 0.6
    #goals[0].pose.position.y = 0.1
    #goals[0].pose.position.z = 0.4
    #goals[0].pose.orientation.x = 0
    #goals[0].pose.orientation.y = 0
    #goals[0].pose.orientation.z = 0.7071
    #goals[0].pose.orientation.w = 0.7071
    #goals[0].gripperOpen = True
    #msg.goals = goals
    
    #print msg.goals
    
    resp = motion_server(msg)
    #print resp.plan.success
    #print '\n', resp.plan.state.world.movable_objects, '\n'
    #record('/home/ragtz/test.bag', [resp.plan])

def putdownTest(action, motion_server, poseGen):
    state = world_state()
    state.world = world_obj()
    
    surf1 = CollisionObject()
    surf1.header = Header()
    surf1.header.frame_id = '/base'
    surf1.id = 'surf1'
    primitive = SolidPrimitive()
    primitive.type = 1
    primitive.dimensions = [1, 1, 0.05]
    surf1.primitives.append(primitive)
    pose = Pose()
    pose.position.x = 1
    pose.position.y = 0
    pose.position.z = 0
    pose.orientation.w = 1
    surf1.primitive_poses.append(pose)
    state.world.surfaces.append(surf1)
    
    obj1 = CollisionObject()
    obj1.header = Header()
    obj1.header.frame_id = '/left_gripper'
    obj1.id = 'obj1'
    primitive = SolidPrimitive()
    primitive.type = 3
    primitive.dimensions = [.25, .02]
    obj1.primitives.append(primitive)
    pose = Pose()
    pose.position.x = 0#.85
    pose.position.y = 0#1.05
    pose.position.z = 0.06#0.32
    #(0.0, -0.7071067811865475, 0.0, 0.7071067811865476)
    pose.orientation.x = 0
    pose.orientation.y = -0.7071067811865475
    pose.orientation.z = 0
    pose.orientation.w = 0.7071067811865476
    obj1.primitive_poses.append(pose)
    state.world.movable_objects.append(obj1)

    state.robot = robot()
    state.robot.id = 'left_arm'
    state.robot.state = RobotState()
    state.robot.state.joint_state.name = ['head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    state.robot.state.joint_state.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    state.robot.state.joint_state.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    state.robot.state.joint_state.effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    msg = motion_plan_parameters()
    msg.state = state
    msg.action = action
    msg.goals = poseGen.next(action, state)
    #print msg.goals
    
    resp = motion_server(msg)
    #print resp.plan.success
    #print resp.plan.state.world.movable_objects
    
if __name__ == "__main__":
    rospy.wait_for_service('motion_server_service')
    motion_server = rospy.ServiceProxy('motion_server_service', motion_service)
    
    poseGen = PoseGenerator()
    
    # (pickup,obj1,left_arm,pose1,pose2)
    for i in range(8):
        pickupTest('(PICKUP,obj1,left_arm,pose1,pose2)', motion_server, poseGen)
    
    # (putdown,obj1,left_arm,pose1,pose2,tloc)
    #for i in range(8):
    #  putdownTest('(PUTDOWN,obj1,left_arm,pose1,pose2,surf1)', motion_server, poseGen)
      
