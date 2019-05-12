#!/usr/bin/env python

import glob
import os
import sys
from collections import deque
import math
import numpy as np
from enum import Enum
import random

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import ai_knowledge as data
from ai_knowledge import Status

# Executor is responsible for moving the vehicle around
# In this implementation it only needs to match the steering and speed so that we arrive at provided waypoints
# BONUS TODO: implement different speed limits so that planner would also provide speed target speed in addition to direction
class Executor(object):
  def __init__(self, knowledge, vehicle):
    self.vehicle = vehicle
    self.knowledge = knowledge
    self.target_pos = knowledge.get_location()
    self.steer = 0.0
  def unit_vector(self,vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)
    
  #Update the executor at some intervals to steer the car in desired direction
  def update(self, time_elapsed):
    status = self.knowledge.get_status()
    #TODO: this needs to be able to handle
    if status == Status.DRIVING:
      destination = self.knowledge.get_current_destination()
      world = self.knowledge.get_world()
      vec = self.vehicle.get_transform().get_forward_vector()
      veh = self.vehicle.get_transform().location

      # world.debug.draw_string(carla.Location(dest.x,dest.y,dest.z), 'O', draw_shadow=False,
      #                                  color=carla.Color(r=255, g=0, b=0), life_time=120.0,
      #                                  persistent_lines=True)
      #debug draw 
      #locationvec = np.array([veh.x, veh.y])
      forwardvec = np.array([vec.x,vec.y]) #rotation vector, one of the vectors we use for our calculation
      nextvec = np.array([destination.x,destination.y])

      v0 = [(destination.x - veh.x), (destination.y - veh.y)] #The other vector from which we gain the angle
      v1 = [vec.x,vec.y] # Rotation vector
      
      print("Forward VEC ", forwardvec)
      print("Destination vector ", nextvec)
        #angle = np.math.atan2(forwardvec,nextvec)
        #angle = np.math.atan2([vec.x,vec.y,vec.z],[dest.x,dest.y,dest.z])
        #angle = np.math.atan2(np.linalg.det([forwardvec,nextvec]),np.dot(forwardvec,nextvec))

      print ("V1 ", v1)
      print ("V_ap ", v0)

      sign = (-v0[0] * vec.y + v0[1] * vec.x) #Calculate the cross product, in order to know which side the vector the angle is

      #cross = np.sign((forwardvec[0] - locationvec[0]) * (nextvec[1] - locationvec[1]) - (forwardvec[1] - locationvec[1]) * (nextvec[0] - locationvec[0]))
      #sign = ((nextvec[0] - locationvec[0]) * (forwardvec[1] - locationvec[1]) - (nextvec[1] - locationvec[1]) * (forwardvec[0] - locationvec[0]))
      #sign = 1
      print("sign is ", sign)

      cosine = np.dot(v1, v0) / np.linalg.norm(v1) / np.linalg.norm(v0)
      angle = np.arccos(cosine) * 90/np.pi # The 90/np.pi is to bind our values to the steering angles, -1 and 1

      #angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
      #angle = np.arccos(np.clip(cosine, -1, 1))
      
      if sign > 0:
        steer = angle
      elif sign < 0:
        steer = (angle * -1) 

      print ("np.degrees are ", np.degrees(angle))
      print("Angle is ", angle)


      throttle = 0.3
      #steer = 0.0
      brake = 0.0


      self.update_control(destination, throttle,  steer, brake, time_elapsed)

  # TODO: steer in the direction of destination and throttle or brake depending on how close we are to destination
  # TODO: Take into account that exiting the crash site could also be done in reverse, 
  #so there might need to be additional data passed between planner and executor, or there needs to be some way to tell this that it is ok to drive in reverse during HEALING and CRASHED states. An example is additional_vars, that could be a list with parameters that can tell us which things we can do (for example going in reverse)
  def update_control(self, destination, throttle, steer, brake, delta_time):
    #calculate throttle and heading
    control = carla.VehicleControl()
    control.throttle = throttle
    control.steer = min(0.7, max(-0.7, steer))
    control.brake = brake
    control.hand_brake = False
    self.vehicle.apply_control(control)

class RoadOption(Enum):
    """
    RoadOption represents the possible topological configurations when moving from a segment of lane to other.
    """
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6

# Planner is responsible for creating a plan for moving around
# In our case it creates a list of waypoints to follow so that vehicle arrives at destination
# Alternatively this can also provide a list of waypoints to try avoid crashing or 'uncrash' itself
class Planner(object):
  def __init__(self, knowledge):
    self.knowledge = knowledge
    self.path = deque([]) 
    self._hop_resolution= 2.0

  # Create a map of waypoints to follow to the destination and save it
  def make_plan(self, source, destination):
    self.path = self.build_path(source,destination)
    self.update_plan()
    self.knowledge.update_destination(self.get_current_destination())
  
  # Function that is called at time intervals to update ai-state
  def update(self, time_elapsed):
    self.update_plan()
    print("Planner update dest ")
    print(self.get_current_destination())
    self.knowledge.update_destination(self.get_current_destination())
  
  #Update internal state to make sure that there are waypoints to follow and that we have not arrived yet
  def update_plan(self):
    if len(self.path) == 0:
      return
    
    if self.knowledge.arrived_at(self.path[0]):
      self.path.popleft()
    
    if len(self.path) == 0:
      self.knowledge.update_status(Status.ARRIVED)
    else:
      self.knowledge.update_status(Status.DRIVING)

  #get current destination 
  def get_current_destination(self):
    status = self.knowledge.get_status()
    #if we are driving, then the current destination is next waypoint
    if status == Status.DRIVING:
      #TODO: Take into account traffic lights and other cars
      print("whhahaaat")
      return self.path[0]
    if status == Status.ARRIVED:
      return self.knowledge.get_location()
    if status == Status.HEALING:
      #TODO: Implement crash handling. Probably needs to be done by following waypoint list to exit the crash site.
      #Afterwards needs to remake the path.
      return self.knowledge.get_location()
    if status == Status.CRASHED:
      #TODO: implement function for crash handling, should provide map of wayoints to move towards to for exiting crash state. 
      #You should use separate waypoint list for that, to not mess with the original path. 
      return self.knowledge.get_location()
    #otherwise destination is same as current position
    return self.knowledge.get_location()

  def getRoadOption(self, current_waypoint, next_waypoint):
    n = next_waypoint.transform.rotation.yaw
    n = n % 360.0

    c = current_waypoint.transform.rotation.yaw
    c = c % 360.0

    diff_angle = (n - c) % 180.0
    if diff_angle < 1.0:
        return RoadOption.STRAIGHT
    elif diff_angle > 90.0: # > ?
        return RoadOption.LEFT
    else:
        return RoadOption.RIGHT

  #TODO: Implementation
  def build_path(self, source, destination):
    self.path = deque([])
    #TODO: create path of waypoints from source 
   
    world = self.knowledge.get_world()   
    map = world.get_map()
    start_waypoint = map.get_waypoint(source.location)
    destLoc = carla.Location(destination.x,destination.y,destination.z)
    end_waypoint = map.get_waypoint(destLoc)
  
    current_waypoint = start_waypoint
    while True :
      w_next = list(current_waypoint.next(5))
      if len(w_next) > 1: 
        min_idx = 0 
        min_dist = None
        for idx, next in enumerate(w_next):
          w_next_next = list(next.next(10))
          # lanechange, both right    left  
          # both right = waypoit.get_right_lane()
          #min_dist = w_next_next[0].transform.location.distance(destLoc)
          for next_next in w_next_next:
            dist = next_next.transform.location.distance(destLoc)
            if min_dist == None:
              min_dist = dist
              min_idx = idx
            elif dist <= min_dist:
              min_dist = dist
              min_idx = idx

        last_waypoint = current_waypoint
        current_waypoint = w_next[min_idx]
      else:
        last_waypoint = current_waypoint
        current_waypoint = w_next[0]



      ropton = self.getRoadOption(last_waypoint,current_waypoint)
      print("Lane------------------")
      print(ropton)
      #print(current_waypoint.get_left_lane())
      #print(current_waypoint.get_right_lane())
      #http://carla.org/2019/03/01/release-0.9.4/
      lanechange_prev= current_waypoint.lane_change # returns carla.LaneChnage
      lanechange= current_waypoint.lane_change # returns carla.LaneChnage
      print("LaneChnage-----------------------")
      print(lanechange)
     
      # 0: None
      # 1: Right
      # 2: Left
      # 3: Both
      if 1 == lanechange and  3 == lanechange_prev or 1 == lanechange and  1 == lanechange_prev:
        current_waypoint = current_waypoint.get_right_lane()
      elif 2 == lanechange and  3 == lanechange_prev or 2 == lanechange and  2 == lanechange_prev:
        current_waypoint = current_waypoint.get_left_lane()

        # 
        # # print(current_waypoint.lanechange())
        # if ropton == RoadOption.LEFT:
        #   print(current_waypoint.get_left_lane())
        #   current_waypoint = current_waypoint.get_left_lane()
        # elif ropton == RoadOption.RIGHT:
        #   print(current_waypoint.get_right_lane())
        #   current_waypoint = current_waypoint.get_right_lane()


    #  or Random
    #  current_waypoint = random.choice(w_next)
      if current_waypoint.transform.location.distance(
                end_waypoint.transform.location) < 5.0: break
      else:
        print(current_waypoint)

        vec =carla.Vector3D(current_waypoint.transform.location.x,current_waypoint.transform.location.y,current_waypoint.transform.location.z)
        self.path.append(vec)    
    
    # vec =carla.Vector3D(current_waypoint.transform.location.x,current_waypoint.transform.location.y,current_waypoint.transform.location.z)
    # self.path.append(vec)
    self.path.append(destination)


    return self.path

