#!/usr/bin/env python

import sys
import rospy
import re
from copy import deepcopy
from task_motion_planner.srv import *
from task_motion_planner.msg import *

class MockMotionPlannerServer(object):
    def __init__(self, surface_dim):
        self.surface_dim = surface_dim
        
    def getMotionPlan(self, req):
        state = req.parameters.state
        world = req.parameters.state.world
        action = re.split(',', req.parameters.action[1:-1])
        goals = req.parameters.goals
        
        #(pickup,obj1,left_arm,pose1,pose2)
        #(putdown,obj1,left_arm,pose1,pose2,tloc)
        
        surfaces = dict()
        for surface in world.surfaces:
            surfaces[surface.id] = [[0 for i in range(self.surface_dim)] for j in range(self.surface_dim)]
            
        for wall in world.walls:
            surface_id = wall.loc.surface_id
            x = wall.loc.x
            y = wall.loc.y
            surfaces[surface_id][y][x] = 1
        
        for movable_object in world.movable_objects:
            surface_id = movable_object.loc.surface_id
            x = movable_object.loc.x
            y = movable_object.loc.y
            grasped = movable_object.loc.grasped
    
            if not grasped:
                surfaces[surface_id][y][x] = 1
    
        res = motion_plan()
        if action[0] == 'PICKUP':
            object_id = goals.object_id
            direction = goals.direction
            
            obj_idx = self._search_for_object(object_id, world.movable_objects)
            if obj_idx == -1:
                print "The object does not exist or is not movable"
                return
            
            obj = world.movable_objects[obj_idx]
            can_pickup = self._can_pickup(obj, direction, surfaces)
            
            if can_pickup:
                state.world.movable_objects[obj_idx].loc.surface_id = ''
                state.world.movable_objects[obj_idx].loc.x = -1
                state.world.movable_objects[obj_idx].loc.y = -1
                state.world.movable_objects[obj_idx].loc.grasped = True
                
                res.state = state
                res.motion = '[picked up ' + obj.id + " from " + direction + "]"
                res.success = True
            else:
                res.state = state
                res.motion = ''
                res.success = False
            
        elif action[0] == 'PUTDOWN':
            object_id = goals.object_id
            goal_surface_id = goals.surface_id
            goal_x = goals.x
            goal_y = goals.y
            
            obj_idx = self._search_for_object(object_id, world.movable_objects)
            if obj_idx == -1:
                print "The object does not exist or is not movable"
                return
                
            obj = world.movable_objects[obj_idx]
            can_putdown = self._can_putdown(obj, surfaces, goal_surface_id, goal_x, goal_y)
            
            if can_putdown:
                state.world.movable_objects[obj_idx].loc.surface_id = goal_surface_id
                state.world.movable_objects[obj_idx].loc.x = goal_x
                state.world.movable_objects[obj_idx].loc.y = goal_y
                state.world.movable_objects[obj_idx].loc.grasped = False
                
                res.state = state
                res.motion = '[put down ' + obj.id + " on surface " + goal_surface_id + " at (" + str(goal_x) + "," + str(goal_y) + ")]"
                res.success = True
            else:
                res.state = state
                res.motion = ''
                res.success = False
            
        else:
            print "Not a valid action"
            return
            
        return res
        
    def _search_for_object(self, obj_name, obj_list):
        for i in range(len(obj_list)):
            if obj_name == obj_list[i].id:
                return i                
        return -1
    
    def _can_pickup(self, obj, direction, surfaces):
        surface_id = obj.loc.surface_id
        x = obj.loc.x
        y = obj.loc.y

        if direction == 'N':
            if y == 0 or surfaces[surface_id][y-1][x] == 0:
                return True
        
        elif direction == 'NE':
            if y == 0:
                if x < self.surface_dim:
                    if surfaces[surface_id][y][x+1] == 0:
                        return True
                else:
                    return True
            elif surfaces[surface_id][y-1][x] == 0 and surfaces[surface_id][y-1][x+1] == 0 and surfaces[surface_id][y][x+1] == 0:
                return True
            
        elif direction == 'E':
            if x == self.surface_dim-1 or surfaces[surface_id][y][x+1] == 0:
                return True
            
        elif direction == 'SE':
            if y == self.surface_dim-1:
                if x < self.surface_dim:
                    if surfaces[surface_id][y][x+1] == 0:
                        return True
                else:
                    return True
            elif surfaces[surface_id][y+1][x] == 0 and surfaces[surface_id][y+1][x+1] == 0 and surfaces[surface_id][y][x+1] == 0:
                return True
            
        elif direction == 'S':
            if y == self.surface_dim-1 or surfaces[surface_id][y+1][x] == 0:
                return True
            
        elif direction == 'SW':
            if y == self.surface_dim-1:
                if x > 0:
                    if surfaces[surface_id][y][x-1] == 0:
                        return True
                else:
                    return True
            elif surfaces[surface_id][y+1][x] == 0 and surfaces[surface_id][y+1][x-1] == 0 and surfaces[surface_id][y][x-1] == 0:
                return True
            
        elif direction == 'W':
            if x == 0 or surfaces[surface_id][y][x-1] == 0:
                return True
            
        elif direction == 'NW':
            if y == 0:
                if x > 0:
                    if surfaces[surface_id][y][x-1] == 0:
                        return True
                else:
                    return True
            elif surfaces[surface_id][y-1][x] == 0 and surfaces[surface_id][y-1][x-1] == 0 and surfaces[surface_id][y][x-1] == 0:
                return True
            
        else:
            print "Not a valid direction"
            
        return False
        
    def _can_putdown(self, obj, surfaces, goal_surface_id, goal_x, goal_y):
        if obj.loc.grasped:
            if surfaces[goal_surface_id][goal_y][goal_x] == 0:
                return True
        else:
            print "Object is not grasped"
        
        return False
    
    def run(self):
        rospy.init_node('motion_planner_server')
        s = rospy.Service('motion_server_service', motion_service, self.getMotionPlan)
        print "Ready to serve motion plans"
        rospy.spin()
    
if __name__ == "__main__":
    motion_planner_server = MockMotionPlannerServer(17)
    motion_planner_server.run()
    
