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
    self.update_speed = 0.5

  def get_speed(self, vehicle):
    """
    Compute speed of a vehicle in Kmh
    :param vehicle: the vehicle for which speed is calculated
    :return: speed as a float in Kmh
    """
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

  #Update the executor at some intervals to steer the car in desired direction
  def update(self, time_elapsed):
    self.update_speed = time_elapsed
    status = self.knowledge.get_status()
    #TODO: this needs to be able to handle
    if status == Status.DRIVING:
      destination = self.knowledge.get_current_destination()
      speed_limit = self.knowledge.retrieve_data("speed_limit")
      
      vec = self.vehicle.get_transform().get_forward_vector()
      veh = self.vehicle.get_transform().location

      self.vehicle.get_world().debug.draw_string(carla.Location(destination.x,destination.y,destination.z), 'O', draw_shadow=False,
                                       color=carla.Color(r=255, g=0, b=0), life_time=120.0,
                                       persistent_lines=True)
   
      forwardvec = np.array([vec.x,vec.y]) #rotation vector, one of the vectors we use for our calculation
      nextvec = np.array([destination.x,destination.y])

      v0 = [(destination.x - veh.x), (destination.y - veh.y)] #The other vector from which we gain the angle
      v1 = [vec.x,vec.y] # Rotation vector
        
      sign = 1
      if np.cross(v0, v1) < 0:
        sign = -1


      cosine = np.dot(v1, v0) / np.linalg.norm(v1) / np.linalg.norm(v0)
      angle = sign * np.arccos(cosine) * 180/np.pi # The 90/np.pi is to bind our values to the steering angles, -1 and 1

      steer = -angle/90
      
      if speed_limit == 0:
        throttle = 0.0
        brake = 1
      else:
        throttle = 0.5
        brake = 0.0
      
      speed = self.get_speed(self.vehicle)
      targetvel = self.knowledge.retrieve_data("targetvel")
      print(speed)
      print(targetvel)
      if speed_limit > speed:
        throttle = targetvel  
      else:
        throttle = targetvel
    


      self.update_control(destination, throttle,  steer, brake, time_elapsed)

  # TODO: steer in the direction of destination and throttle or brake depending on how close we are to destination
  # TODO: Take into account that exiting the crash site could also be done in reverse, 
  #so there might need to be additional data passed between planner and executor, or there needs to be some way to tell this that it is ok to drive in reverse during HEALING and CRASHED states. An example is additional_vars, that could be a list with parameters that can tell us which things we can do (for example going in reverse)
  def update_control(self, destination, throttle, steer, brake, delta_time):
    #calculate throttle and heading
    control = carla.VehicleControl()
    control.throttle = throttle
    control.steer = min(1, max(-1, steer))
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
    self.knowledge.update_data('end_destination', destination)
    self.knowledge.update_destination(self.get_current_destination())
  
  # Function that is called at time intervals to update ai-state
  def update(self, time_elapsed):
    self.update_plan()
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
   
    map = self.knowledge.retrieve_data("map")
    start_waypoint = map.get_waypoint(source.location)
    destLoc = carla.Location(destination.x,destination.y,destination.z)
    end_waypoint = map.get_waypoint(destLoc)
  
    current_waypoint = start_waypoint
    while True :
      w_next = list(current_waypoint.next(10))
      if len(w_next) > 1: 
        min_idx = 0 
        min_dist = None
        for idx, next in enumerate(w_next):
          w_next_next = list(next.next(20))
     
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
        #check 



    
      #print(current_waypoint.get_left_lane())
      #print(current_waypoint.get_right_lane())
  
      #http://carla.org/2019/03/01/release-0.9.4/
      
        lanechange= current_waypoint.lane_change # returns carla.LaneChnage
        print("LaneChange-----------------------")
        print(lanechange)
       
        # 0: None
        # 1: Right
        # 2: Left
        # 3: Both

        if carla.LaneChange.Right == lanechange or carla.LaneChange.Both == lanechange:
          print("LaneChangeRight-----------------------")
          print(current_waypoint.lane_change)
          print(current_waypoint.get_right_lane())
          right_candidate = current_waypoint.get_right_lane()
          if right_candidate.transform.location.distance(end_waypoint.transform.location) < current_waypoint.transform.location.distance(
                end_waypoint.transform.location):
            current_waypoint = right_candidate
            
        elif carla.LaneChange.Left == lanechange or carla.LaneChange.Both == lanechange:
          print("LaneChangeLeft-----------------------")
          print(current_waypoint.get_left_lane())
          left_candidate = current_waypoint.get_left_lane()
          if left_candidate.transform.location.distance(end_waypoint.transform.location) < current_waypoint.transform.location.distance(
                end_waypoint.transform.location):
            current_waypoint = left_candidate




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

