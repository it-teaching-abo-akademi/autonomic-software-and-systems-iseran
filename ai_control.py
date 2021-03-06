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
import ai_test as ai_test
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

  def get_speed(self, vehicle):
    # to km/h
    vel = vehicle.get_velocity()
    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

  #Update the executor at some intervals to steer the car in desired direction
  #Updates the location, speed_limit, angle, at every frame. Gets the information from the additional_vars in update_control
  def update(self, time_elapsed):
    self.update_speed = time_elapsed
    status = self.knowledge.get_status()
    #print("Printing the status", status)
    #TODO: this needs to be able to handle
    destination = self.knowledge.get_current_destination()

    if status == Status.DRIVING:

      vec = self.vehicle.get_transform().get_forward_vector()
      veh = self.vehicle.get_transform().location
      
      #Debug
      self.vehicle.get_world().debug.draw_string(carla.Location(destination.x,destination.y,destination.z), 'O', draw_shadow=False,
                                       color=carla.Color(r=255, g=0, b=0), life_time=120.0,
                                       persistent_lines=True)

      #rotation vector, one of the vectors we use for our calculation
      forwardvec = np.array([vec.x,vec.y]) 
      #The other vector from which we gain the angle
      nextvec = np.array([destination.x,destination.y])

      #Destination vector
      dest_vec = [(destination.x - veh.x), (destination.y - veh.y)] 
      # Rotation vector
      fwd_vec = [vec.x,vec.y] 
        
      sign = 1
      if np.cross(dest_vec, fwd_vec) < 0:
        sign = -1
      
      #calculate angle
      cosine = np.dot(fwd_vec, dest_vec) / np.linalg.norm(fwd_vec) / np.linalg.norm(dest_vec)
      angle = sign * np.arccos(cosine) * 180/np.pi # The 90/np.pi is to bind our values to the steering angles, -1 and 1

      steer = -angle/90
      speed_limit = self.knowledge.retrieve_data("speed_limit")
      if speed_limit == 0:
        throttle = 0.0
        brake = 1
      else:
        #throttle = 0.5
        brake = 0.0
      
      #V1 stupid speed version, but this works betther for keeping around speed limit
      speed = self.get_speed(self.vehicle)
      if speed_limit > speed:
        throttle = 1  
        brake = 0.0
      else:
        throttle = 0.0

      #V2  Speed with calculated velocity to next waypoint 
      # comment out if testing V1
      vel = self.vehicle.get_velocity()
      speed = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
      targetvel = self.knowledge.retrieve_data("targetvel")
      const=1
      if targetvel > speed:
        throttle = min((targetvel-speed)/const,1)  
        brake = 0.0
      else:
        throttle = 0.0
      # comment out if testing V1
    else:
      # break if arrived 
      throttle = 0 
      brake = 1
      steer = 0

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
# Create a plan of waypoints from the start to destination, before we start driving. A bit like Dijkstra algorithm
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
    
    #create path of waypoints from source 
   
    map = self.knowledge.retrieve_data("map")
    start_waypoint = map.get_waypoint(source.location)
    # convert to locatio to be able to compare distance 
    destLoc = carla.Location(destination.x,destination.y,destination.z)
    end_waypoint = map.get_waypoint(destLoc)
  
    current_waypoint = start_waypoint
    while True :
      #find nearest waaypoints 
      w_next = list(current_waypoint.next(10))
      if len(w_next) > 1: 
        min_idx = 0 
        min_dist = None
        # for every waypoint check if there is a waypoint closer to the destination
        # use the closest waypoint index for next waypoint from W_next 
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

        current_waypoint = w_next[min_idx]
      else:
        # if only one we use it 
        current_waypoint = w_next[0]

        # get lane chagen 
        lanechange= current_waypoint.lane_change # returns carla.LaneChange  
      
        # TODO modify so it has a candidate and it is compared in the second if!
        if carla.LaneChange.Right == lanechange or carla.LaneChange.Both == lanechange:
          #print("LaneChangeRight-----------------------")
          right_candidate = current_waypoint.get_right_lane()
          # if candicate is closer than choosen waypoint
          if right_candidate.transform.location.distance(end_waypoint.transform.location) < current_waypoint.transform.location.distance(
                end_waypoint.transform.location):
            current_waypoint = right_candidate
            
        elif carla.LaneChange.Left == lanechange or carla.LaneChange.Both == lanechange:
          #print("LaneChangeLeft-----------------------")
          left_candidate = current_waypoint.get_left_lane()
          # if candicate is closer than choosen waypoint 
          if left_candidate.transform.location.distance(end_waypoint.transform.location) < current_waypoint.transform.location.distance(
                end_waypoint.transform.location):
            current_waypoint = left_candidate


      # break if we are 5 from destination
      if current_waypoint.transform.location.distance(
                end_waypoint.transform.location) < 5.0: break
      else:
        #print(current_waypoint)
        # convert to vector
        vec =carla.Vector3D(current_waypoint.transform.location.x,current_waypoint.transform.location.y,current_waypoint.transform.location.z)
        self.path.append(vec)    

    self.path.append(destination)
    return self.path

