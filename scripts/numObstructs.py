#!/usr/bin/env python

from task_motion_planner.msg import *

# world is a world msg from generateWorldMsg
# surface dim --> size of grid (default 17)
def numObstructsWorld(world, surface_dim):
    world = world.world
    
    surfaces = dict()
    for surface in world.surfaces:
        surfaces[surface.id] = [[0 for i in range(surface_dim)] for j in range(surface_dim)]
    
    for movable_object in world.movable_objects:
        surface_id = movable_object.loc.surface_id
        x = movable_object.loc.x
        y = movable_object.loc.y
        grasped = movable_object.loc.grasped

        if not grasped:
            surfaces[surface_id][y][x] = 1
            
    numObs = 0
    for movable_object in world.movable_objects:
        numObs += numObstructsObject(movable_object, surface_dim, surfaces)
            
    return numObs
            
def numObstructsObject(obj, surface_dim, surfaces):
    numObs = 0
    numObs += obstructsN(obj, surface_dim, surfaces)
    numObs += obstructsNE(obj, surface_dim, surfaces)
    numObs += obstructsE(obj, surface_dim, surfaces)
    numObs += obstructsSE(obj, surface_dim, surfaces)
    numObs += obstructsS(obj, surface_dim, surfaces)
    numObs += obstructsSW(obj, surface_dim, surfaces)
    numObs += obstructsW(obj, surface_dim, surfaces)
    numObs += obstructsNW(obj, surface_dim, surfaces)
    return numObs
    
def obstructsN(obj, surface_dim, surfaces):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if y == 0 or surfaces[surface_id][y-1][x] == 0:
        return False
    else:
        return True
    
def obstructsNE(obj, surface_dim, surfaces):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if y == 0:
        # cell to the right empty
        if x < surface_dim-1 and surfaces[surface_id][y][x+1] == 0:
                return False
        else:
            return False
    elif x < surface_dim-1:
        # above, above & right, right
        if surfaces[surface_id][y-1][x] == 0 and surfaces[surface_id][y-1][x+1] == 0 and surfaces[surface_id][y][x+1] == 0:
            return False
        else:
            return True
    elif x == 0:
        return not surfaces[surface_id][y-1][x] == 0
    else:
        return True
    
def obstructsE(obj, surface_dim, surfaces):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if x == surface_dim-1 or surfaces[surface_id][y][x+1] == 0:
        return False
    else:
        return True
    
def obstructsSE(obj, surface_dim, surfaces):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    # y is bottom
    if y == surface_dim-1:
        # cells to the right
        if x < surface_dim-1:
            # no block to the right?
            return not surfaces[surface_id][y][x+1] == 0
        else:
            return False
    elif x < surface_dim-1:
        return not (surfaces[surface_id][y+1][x] == 0 and surfaces[surface_id][y+1][x+1] == 0 and surfaces[surface_id][y][x+1] == 0)
    elif x == surface_dim-1:
        return not surfaces[surface_id][y+1][x] == 0
    else:
        return True

def obstructsS(obj, surface_dim, surfaces):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if y == surface_dim-1 or surfaces[surface_id][y+1][x] == 0:
        return False
    else:
        return True
    
def obstructsSW(obj, surface_dim, surfaces):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    # y is bottom
    if y == surface_dim-1:
        if x > 0:
            return not surfaces[surface_id][y][x-1] == 0
        else:
            return False
    elif x > 0:
        return not (surfaces[surface_id][y+1][x] == 0 and surfaces[surface_id][y+1][x-1] == 0 and surfaces[surface_id][y][x-1] == 0)
    elif x == 0:
        return not surfaces[surface_id][y+1][x] == 0
    else:
        return True
    
def obstructsW(obj, surface_dim, surfaces):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    if x == 0 or surfaces[surface_id][y][x-1] == 0:
        return False
    else:
        return True
    
def obstructsNW(obj, surface_dim, surfaces):
    surface_id = obj.loc.surface_id
    x = obj.loc.x
    y = obj.loc.y
    
    # top row
    if y == 0:
        # cells to the left
        if x > 0:
            return not surfaces[surface_id][y][x-1] == 0
        else:
            return False
    elif x < surface_dim-1:
        return not (surfaces[surface_id][y-1][x] == 0 and surfaces[surface_id][y-1][x-1] == 0 and surfaces[surface_id][y][x-1] == 0)
    elif x == surface_dim-1:
        return not surfaces[surface_id][y-1][x] == 0
    else:
        return True