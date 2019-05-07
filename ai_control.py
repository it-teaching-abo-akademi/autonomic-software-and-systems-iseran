#!/usr/bin/env python

import glob
import os
import sys
from collections import deque
import math
import numpy as np

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
    
  #Update the executor at some intervals to steer the car in desired direction
  #Updates the location, speed_limit, angle, at every frame. Gets the information from the additional_vars in update_control
  def update(self, time_elapsed):
    status = self.knowledge.get_status()
    #print("Printing the status", status)
    #TODO: this needs to be able to handle
    if status == Status.DRIVING:
      dest = self.knowledge.get_current_destination()
      #print("dest is ", dest)
      self.update_control(dest, [1], time_elapsed)

  # TODO: steer in the direction of destination and throttle or brake depending on how close we are to destination
  # TODO: Take into account that exiting the crash site could also be done in reverse, so there might need to be additional data passed between planner and 
	#executor, or there needs to be some way to tell this that it is ok to drive in reverse during HEALING and CRASHED states. An example is additional_vars, 
	#that could be a list with parameters that can tell us which things we can do (for example going in reverse)
  #*
  # The goal here is to use additional_vars as a list of different parameters, which control the vehicles speed, angle and location
  # Since the update function is called every frame, the vehicle be controlled with the help of speed_limit and angle
  # Determining the appropirate speed and such can be done with the carla python api, for example Waypoint.is_intersection 
  # (should slow down at intersections or stop completely)
  # 
  # We should first implement the make_plan function, because we can't control speed and such before making if we don't know where we are going 
  # additional_vars can contain variables such as speed_limit, angle, location 
  # *#
  #speed_limit, angle, location,
  def update_control(self, destination, additional_vars, delta_time):

    #calculate throttle and heading
    control = carla.VehicleControl()
    control.throttle = 0.8
    control.steer = 0.0
    control.brake = 0.0
    control.hand_brake = False
    self.vehicle.apply_control(control)

# Planner is responsible for creating a plan for moving around
# In our case it creates a list of waypoints to follow so that vehicle arrives at destination
# Alternatively this can also provide a list of waypoints to try avoid crashing or 'uncrash' itself
# Create a plan of waypoints from the start to destination, before we start driving. A bit like Dijkstra algorithm
class Planner(object):
  def __init__(self, knowledge, world):
    self.knowledge = knowledge
    self.path = deque([])

  # Create a map of waypoints to follow to the destination and save it
  def make_plan(self, source, destination):
    self.path = self.build_path(source,destination)
    self.update_plan()
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

  #*
  # We have to build a path with the help of the waypoints in the map.
  # So we should do the following:
  # 1. Get all the waypoints close to our spawned vehicle, which means that we need the vehicles position and and the surronding waypoints
  # 2. Get the surronding waypoints with the help of Dijkstra algorithm (something like that)
  # 3. Make a path to the destination, by comparing the distance between the from the waypoints to the main destination
  # 4. ???
  # 5. Profit??
  # *#
  
  #TODO: Implementation
  def build_path(self, source, destination):
    self.path = deque([])
    self.path.append(destination)
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    map = world.get_map()
    actor_list = world.get_actors()
    #spawn_points = world.get_map().get_spawn_points()
    #actor = actor_list.find(id)
    
    for vehicle in actor_list.filter('vehicle.*'):
      print ("Vehicle is at : ", vehicle.get_location())
      vehLocation = vehicle.get_location()

    waypoint = map.get_waypoint(vehicle.get_location())
    print ("waypoint is ", waypoint)
    for x in range(10):
      next = list(waypoint.next(5.0))
      print ("waypoints close by are ", next)
      #vehicle.set_transform(waypoint.transform)
      
    #waypoint = map.get_waypoint(vehicle.get_location())
    #source = carla.Map('name')
    #print ("source is ", source)
    #print ("destination is ", destination)
    #print ("waypoint ", waypoint)
    #print ("actor_list is ", actor_list)
    #TODO: create path of waypoints from source to destination
    #print ("location is ",source.location)
    return self.path
