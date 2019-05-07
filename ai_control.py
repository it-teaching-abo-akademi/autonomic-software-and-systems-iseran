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
  def unit_vector(self,vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)
    
  #Update the executor at some intervals to steer the car in desired direction
  def update(self, time_elapsed):
    status = self.knowledge.get_status()
    #TODO: this needs to be able to handle
    if status == Status.DRIVING:
      dest = self.knowledge.get_current_destination()

      #debug draw 
      #
      vec = self.vehicle.get_transform().get_forward_vector()
      forwardvec = np.array([vec.x,vec.y])
      nextvec = np.array([dest.x,dest.y])
      both = np.array([forwardvec,nextvec])
      #angle = np.math.atan2(forwardvec,nextvec)
      #angle = np.math.atan2([vec.x,vec.y,vec.z],[dest.x,dest.y,dest.z])
      #angle = np.math.atan2(np.linalg.det([forwardvec,nextvec]),np.dot(forwardvec,nextvec))
      v1 = [vec.x,vec.y]
      v2 = [dest.x,dest.y]
      v1_u = self.unit_vector(v1)
      v2_u = self.unit_vector(v2)
      angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
      # print(v1)
      # print(v2)
      # cosang = np.dot(v1, v2) # dotproduct
      # sinang = np.linalg.norm(np.cross(v1, v2)) # normalize 
      # angle = np.arccos(sinang, cosang)
      print (np.degrees(angle))
      print (angle)
      print("Moving")
      if angle < 180:
        steer = 0.5
      else:
        steer = -0.5

      throttle = 0.5
      #steer = 0.0
      brake = 0.0


      self.update_control(dest, throttle, steer, brake, time_elapsed)

  # TODO: steer in the direction of destination and throttle or brake depending on how close we are to destination
  # TODO: Take into account that exiting the crash site could also be done in reverse, 
  #so there might need to be additional data passed between planner and executor, or there needs to be some way to tell this that it is ok to drive in reverse during HEALING and CRASHED states. An example is additional_vars, that could be a list with parameters that can tell us which things we can do (for example going in reverse)
  def update_control(self, destination, throttle, steer, brake, delta_time):
    #calculate throttle and heading
    control = carla.VehicleControl()
    control.throttle = throttle
    control.steer = steer
    control.brake = brake
    control.hand_brake = False
    self.vehicle.apply_control(control)



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
          # both left = waypoit.get_left_lane()
          #min_dist = w_next_next[0].transform.location.distance(destLoc)
          for next_next in w_next_next:
            dist = next_next.transform.location.distance(destLoc)
            if min_dist == None:
              min_dist = dist
              min_idx = idx
            elif dist <= min_dist:
              min_dist = dist
              min_idx = idx

        current_waypoint = w_next[min_idx]
      else:
        current_waypoint = w_next[0]
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

