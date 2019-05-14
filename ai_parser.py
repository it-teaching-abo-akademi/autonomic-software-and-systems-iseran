#!/usr/bin/env python

import glob
import os
import sys
import math

try:
    sys.path.append(glob.glob('**/*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import weakref
import carla
import ai_knowledge as data


# Monitor is responsible for reading the data from the sensors and telling it to the knowledge
# TODO: Implement other sensors (lidar and depth sensors mainly)
# TODO: Use carla API to read whether car is at traffic lights and their status, update it into knowledge
class Monitor(object):
  def __init__(self, knowledge,vehicle):
    self.vehicle = vehicle
    self.knowledge = knowledge
    weak_self = weakref.ref(self)
    
    self.knowledge.update_data('location', self.vehicle.get_transform().location)
    self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)


    world = self.vehicle.get_world()
    self.knowledge.update_data('map', world.get_map())
    self.knowledge.update_data('speed_limit', vehicle.get_speed_limit())
    self.knowledge.update_data('at_lights', vehicle.is_at_traffic_light())
    self.knowledge.update_data('traffic_light', self.vehicle.get_traffic_light())
    self.knowledge.update_data('velocity', self.vehicle.get_velocity())
    self.knowledge.update_data('targetvel', 4)


    bp = world.get_blueprint_library().find('sensor.other.lane_detector')
    self.lane_detector = world.spawn_actor(bp, carla.Transform(), attach_to=self.vehicle)
    self.lane_detector.listen(lambda event: Monitor._on_invasion(weak_self, event))

  #Function that is called at time intervals to update ai-state
  def update(self, time_elapsed):
    # Update the position of vehicle into knowledge
    self.knowledge.update_data('speed_limit', self.vehicle.get_speed_limit())
    self.knowledge.update_data('at_lights', self.vehicle.is_at_traffic_light())
    self.knowledge.update_data('traffic_light', self.vehicle.get_traffic_light())
    self.knowledge.update_data('location', self.vehicle.get_transform().location)
    self.knowledge.update_data('rotation', self.vehicle.get_transform().rotation)
    self.knowledge.update_data('velocity', self.vehicle.get_velocity())


  @staticmethod
  def _on_invasion(weak_self, event):
    self = weak_self()
    if not self:
      return
    self.knowledge.update_data('lane_invasion',event.crossed_lane_markings)

# Analyser is responsible for parsing all the data that the knowledge has received from Monitor and turning it into something usable
# TODO: During the update step parse the data inside knowledge into information that could be used by planner to plan the route
class Analyser(object):
  def __init__(self, knowledge):
    self.knowledge = knowledge

  #Function that is called at time intervals to update ai-state
  def update(self, time_elapsed):
    ## find out if the car is at a trafic light and light red  
    # change speed to 0 
    if self.knowledge.retrieve_data("at_lights"):
      traffic_light = self.knowledge.retrieve_data("traffic_light")
      if traffic_light.state == carla.TrafficLightState.Red:
        self.knowledge.update_data('speed_limit', 0)
    #velocity calculation depending on destination and speed
    # distance to target 
    targetvel = 4 # deafult somhow?? 
    location = carla.Vector3D(self.knowledge.retrieve_data("location").x,self.knowledge.retrieve_data("location").y, self.knowledge.retrieve_data("location").z)
    end_destination = self.knowledge.retrieve_data("end_destination")
    # if close to destination 
    if self.knowledge.distance(location,end_destination) < 4:
      targetvel = min(self.knowledge.distance(location,end_destination),4)
    else:
      vel = self.knowledge.retrieve_data("velocity")
      print("________________________________________")
      vel =  vel.z #M/s   3.6math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
      print(vel)
      targetvel = min(max((targetvel-vel)-0.5,0),1)
      print(targetvel)

    #set target velocity into knowledge
    self.knowledge.update_data('targetvel',targetvel)


    return
