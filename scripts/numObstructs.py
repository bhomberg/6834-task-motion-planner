#!/usr/bin/env python

from task_motion_planner.msg import *

# world is a world msg from generateWorldMsg
# surface dim --> size of grid (default 17)
def numObstructsWorld(world, surface_dim):
    surfaces = world.world.surfaces
    movable_objects = world.world.movable_objects
    
    surfaces = dict()
    for surface in surfaces:
        surfaces[surface.id] = [[0 for i in range(surface_dim)] for j in range(surface_dim)]
    
    for movable_object in movable_objects:
        surface_id = movable_object.loc.surface_id
        x = movable_object.loc.x
        y = movable_object.loc.y
        grasped = movable_object.loc.grasped

        if not grasped:
            surfaces[surface_id][y][x] = 1
            
    numObs = 0
    for surface in surfaces:
        for movable_object in movable_objects:
            numObs += numObstructsObject(surfaces, movable_object)
            
    return numObs
            
def numObstructsObject(surfaces, obj):
    numObs = 0
    numObs += obstructsN(surfaces, obj)
    numObs += obstructsNE(surfaces, obj)
    numObs += obstructsE(surfaces, obj)
    numObs += obstructsSE(surfaces, obj)
    numObs += obstructsS(surfaces, obj)
    numObs += obstructsSW(surfaces, obj)
    numObs += obstructsW(surfaces, obj)
    numObs += obstructsNW(surfaces, obj)
    return numObs
    
def obstructsN(surfaces, obj):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if y == 0 or surfaces[surface_id][y-1][x] == 0:
        return False
    else:
        return True
    
def obstructsNE(surfaces, obj):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if y == 0:
        if x < self.surface_dim:
            if surfaces[surface_id][y][x+1] == 0:
                return False
        else:
            return False
    elif surfaces[surface_id][y-1][x] == 0 and surfaces[surface_id][y-1][x+1] == 0 and surfaces[surface_id][y][x+1] == 0:
        return False
    else:
        return True
    
def obstructsE(surfaces, obj):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if x == self.surface_dim-1 or surfaces[surface_id][y][x+1] == 0:
        return False
    else:
        return True
    
def obstructsSE(surfaces, obj):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if y == self.surface_dim-1:
        if x < self.surface_dim:
            if surfaces[surface_id][y][x+1] == 0:
                return False
        else:
            return False
    elif surfaces[surface_id][y+1][x] == 0 and surfaces[surface_id][y+1][x+1] == 0 and surfaces[surface_id][y][x+1] == 0:
        return False
    else:
        return True
    
def obstructsS(surfaces, obj):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if y == self.surface_dim-1 or surfaces[surface_id][y+1][x] == 0:
        return False
    else:
        return True
    
def obstructsSW(surfaces, obj):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if y == self.surface_dim-1:
        if x > 0:
            if surfaces[surface_id][y][x-1] == 0:
                return False
        else:
            return False
    elif surfaces[surface_id][y+1][x] == 0 and surfaces[surface_id][y+1][x-1] == 0 and surfaces[surface_id][y][x-1] == 0:
        return False
    else:
        return True
    
def obstructsW(surfaces, obj):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if x == 0 or surfaces[surface_id][y][x-1] == 0:
        return False
    else:
        return True
    
def obstructsNW(surfaces, obj):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if y == 0:
        if x > 0:
            if surfaces[surface_id][y][x-1] == 0:
                return False
        else:
            return False
    elif surfaces[surface_id][y-1][x] == 0 and surfaces[surface_id][y-1][x-1] == 0 and surfaces[surface_id][y][x-1] == 0:
        return False
    else:
        return True
    
