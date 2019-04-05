#!/usr/bin/env python
# Copyright (c) 2018-2019 Intel Labs, 2019 FedorChervinskii
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
CARLA Challenge Evaluator

Provisional code to evaluate Autonomous Agents for the CARLA Autonomous Driving challenge
"""

from __future__ import print_function
from collections import namedtuple
from py_trees.common import Status
import random
import time

import carla

from srunner.challenge.envs.sensor_interface import CallBack, Speedometer, HDMapReader, SensorInterface
from srunner.scenarios.challenge_very_basic import ChallengeVeryBasic
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenarios.config_parser import ScenarioConfiguration
from srunner.scenariomanager.scenario_manager import ScenarioManager

ActorConfiguration = namedtuple("ActorConfiguration", ["pos_x", "pos_y", "pos_z", "yaw",
                                                       "random_location", "autopilot", "transform",
                                                       "model"])
TargetConfiguration = namedtuple("TargetConfiguration", ["pos_x", "pos_y", "pos_z",
                                                         "transform"])

DEFAULT_ACTOR = ActorConfiguration(None, None, None, None, True, True, None, "vehicle.*")


class RandomTargetRunner(object):

    """
    TBD
    """

    ego_vehicle = None
    actors = []

    # Tunable parameters
    client_timeout = 5.0  # in seconds
    wait_for_world = 10.0

    # CARLA world and scenario handlers
    world = None
    manager = None

    def __init__(self, host, port, num_vehicles=20, debug=False):
        self.num_vehicles = num_vehicles
        self.output_scenario = []
        self._sensors_list = []
        self._hop_resolution = 2.0
        self.target = None
        self.spawn_point = None
        self.sensor_interface = None
        self.debug = debug
        self.client = carla.Client(host, int(port))
        self.client.set_timeout(self.client_timeout)
        self.start()

    def __del__(self):
        """
        Cleanup and delete actors, ScenarioManager and CARLA world
        """

        self.cleanup(True)
        if self.manager is not None:
            del self.manager
        if self.world is not None:
            del self.world

    def cleanup(self, ego=False):
        """
        Remove and destroy all actors
        """

        # We need enumerate here, otherwise the actors are not properly removed
        for i, _ in enumerate(self.actors):
            if self.actors[i] is not None:
                self.actors[i].destroy()
                self.actors[i] = None
            print(self.actors)
        self.actors = []

        for i, _ in enumerate(self._sensors_list):
            if self._sensors_list[i] is not None:
                self._sensors_list[i].destroy()
                self._sensors_list[i] = None
        self._sensors_list = []

        if self.manager is not None:
            del self.manager      
        
        if ego and self.ego_vehicle is not None:
            self.ego_vehicle.destroy()
            self.ego_vehicle = None

    def setup_ego_vehicle_and_target(self, model, min_distance_to_goal, max_distance_to_goal):
        blueprint_library = self.world.get_blueprint_library()

        # Get vehicle by model
        blueprint = random.choice(blueprint_library.filter(model))
        blueprint.set_attribute('role_name', 'hero')
        spawn_points = list(self.world.get_map().get_spawn_points())
        random.shuffle(spawn_points)
        vehicle = None

        spawn_point = None
        for spawn_point in spawn_points:
            vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
            if vehicle:
                self.spawn_point = spawn_point
                target = None
                for target in spawn_points:
                    if min_distance_to_goal < spawn_point.location.distance(target.location) < max_distance_to_goal:
                        self.target = target
                        break
                if target is None:
                    raise Exception("Error: Unable to find a satisfying goal")
                break

        if vehicle is None:
            raise Exception(
                "Error: Unable to spawn vehicle {} at {}".format(model, spawn_point))
        else:
            # Let's deactivate the autopilot of the vehicle
            vehicle.set_autopilot(False)

        return vehicle

    def setup_vehicle(self, model, spawn_point, hero=False, autopilot=False, random_location=False):
        """
        Function to setup the most relevant vehicle parameters,
        incl. spawn point and vehicle model.
        """

        vehicle = None
        blueprint_library = self.world.get_blueprint_library()

        # Get vehicle by model
        blueprint = random.choice(blueprint_library.filter(model))
        if hero:
            blueprint.set_attribute('role_name', 'hero')
        else:
            blueprint.set_attribute('role_name', 'scenario')

        if random_location:
            spawn_points = list(self.world.get_map().get_spawn_points())
            random.shuffle(spawn_points)
            for spawn_point in spawn_points:
                vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
                if vehicle:
                    break
        else:
            vehicle = self.world.try_spawn_actor(blueprint, spawn_point)

        if vehicle is None:
            raise Exception(
                "Error: Unable to spawn vehicle {} at {}".format(model, spawn_point))
        else:
            # Let's deactivate the autopilot of the vehicle
            vehicle.set_autopilot(autopilot)

        return vehicle

    def setup_sensors(self, sensors, vehicle):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param sensors: list of sensors
        :param vehicle: ego vehicle
        :return:
        """
        self.sensor_interface = SensorInterface()
        bp_library = self.world.get_blueprint_library()
        for sensor_spec in sensors:
            # These are the pseudosensors (not spawned)
            if sensor_spec['type'].startswith('sensor.speedometer'):
                # The speedometer pseudo sensor is created directly here
                sensor = Speedometer(vehicle, sensor_spec['reading_frequency'])
            elif sensor_spec['type'].startswith('sensor.hd_map'):
                # The HDMap pseudo sensor is created directly here
                sensor = HDMapReader(vehicle, sensor_spec['reading_frequency'])
            # These are the sensors spawned on the carla world
            else:
                sensor_location, sensor_rotation = None, None
                bp = bp_library.find(sensor_spec['type'])
                if sensor_spec['type'].startswith('sensor.camera'):
                    bp.set_attribute('image_size_x', str(sensor_spec['width']))
                    bp.set_attribute('image_size_y', str(sensor_spec['height']))
                    bp.set_attribute('fov', str(sensor_spec['fov']))
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.lidar'):
                    bp.set_attribute('range', '5000')
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.other.gnss'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation()
                else:
                    raise NotImplementedError

                # create sensor
                sensor_transform = carla.Transform(sensor_location, sensor_rotation)
                sensor = self.world.spawn_actor(bp, sensor_transform,
                                                vehicle)
            # setup callback
            sensor.listen(CallBack(sensor_spec['id'], sensor, self.sensor_interface))
            self._sensors_list.append(sensor)

        # check that all sensors have initialized their data structure
        while not self.sensor_interface.all_sensors_ready():
            time.sleep(0.1)

    def prepare_actors(self):
        """
        Spawn or update all scenario actors
        """

        # If ego_vehicle already exists, just update location
        # Otherwise spawn ego vehicle
        if self.ego_vehicle is None:
            self.ego_vehicle = self.setup_ego_vehicle_and_target("vehicle.lincoln.mkz2017",
                                                                 min_distance_to_goal=10,
                                                                 max_distance_to_goal=100)
        else:
            raise NotImplementedError

        sensors = [
            {'type': 'sensor.other.gnss', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'id': 'GPS'},
        ]

        # setup sensors
        self.setup_sensors(sensors, self.ego_vehicle)

        # spawn all other actors
        self.actors = []
        for i in range(self.num_vehicles):
            new_actor = self.setup_vehicle(DEFAULT_ACTOR.model, DEFAULT_ACTOR.transform, hero=False, autopilot=True,
                                           random_location=True)
            if new_actor is not None:
                self.actors.append(new_actor)
            
    def analyze_scenario(self):
        """
        Provide feedback about success/failure of a scenario
        """
        result, score, return_message = self.manager.analyze_scenario_challenge()
        self.output_scenario.append((result, score, return_message))

        # show results stoud
        print(return_message)

    def final_summary(self):
        return_message = ""

        total_scenarios = len(self.output_scenario)
        total_score = 0.0
        for item in self.output_scenario:
            total_score += item[1] / float(total_scenarios)
            return_message += ("\n" + item[2])

        avg_message = "\n==================================\n==[Avg. score = {:.2f}]".format(total_score)
        avg_message += "\n=================================="
        return_message = avg_message + return_message
        print(return_message)

    def start(self):
        """
        Run random target simulator
        """
        print("starting")
        config = ScenarioConfiguration()
        config.town = "Town01"
        self.world = self.client.get_world()
        print("!!!")
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)
        self.world = self.client.load_world(config.town)
#        # Wait for the world to be ready
        self.world.wait_for_tick(self.wait_for_world)
        print("world ready!")
    
        # Create scenario manager
        self.manager = ScenarioManager(self.world, self.debug)

        config.name = "RandomSpawnRandomTarget"
        config.type = "ChallengeBasic"

        if True:
            self.prepare_actors()
            for actor in self.actors:
                config.other_actors.append(DEFAULT_ACTOR)
            pos_x, pos_y, pos_z = self.spawn_point.location.x, self.spawn_point.location.y, self.spawn_point.location.z
            config.ego_vehicle = ActorConfiguration(pos_x, pos_y, pos_z,
                                                    self.spawn_point.rotation.yaw,
                                                    False, False,
                                                    carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z)),
                                                    "vehicle.*")
            fin_x, fin_y, fin_z = self.target.location.x, self.target.location.y, self.target.location.z
            config.target = TargetConfiguration(fin_x, fin_y, fin_z,
                                                carla.Transform(carla.Location(x=fin_x, y=fin_y, z=fin_z)))
            distance = self.spawn_point.location.distance(self.target.location)
            min_average_speed_towards_goal = 1.  # m/s
            timeout = distance / min_average_speed_towards_goal
            scenario = ChallengeVeryBasic(self.world,
                                          self.ego_vehicle,
                                          self.actors,
                                          config.town,
                                          randomize=False,
                                          debug_mode=self.debug,
                                          config=config,
                                          timeout=timeout)

            # Load scenario and run it
            self.manager.load_scenario(scenario)
            self.manager.start_scenario()
        
        # Switiching into synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = False
        self.world.apply_settings(settings)

    def get_observation_dict(self):
#       input_data = self.sensor_interface.get_data()
        location = CarlaDataProvider.get_location(self.ego_vehicle)
        velocity = CarlaDataProvider.get_velocity(self.ego_vehicle)
        yaw = self.manager.ego_vehicle.get_transform().rotation.yaw
        if location is not None:
            observation_dict = {"location": location,
                                "goal": self.target.location,
                                "yaw": yaw,
                                "speed": velocity}
        else:
            observation_dict = None
        print(observation_dict)
        return observation_dict
        
    def step(self, action):
        observation_dict, done = None, False
        if self.manager._running:
            control = carla.VehicleControl()
            control.steer = float(action[0])
            control.throttle = float(0.5 + action[1] / 2.)
            control.brake = float(0.) #float(action[2])
            control.hand_brake = False
            control.manual_gear_shift = False
            print("step with action: {}".format([control.steer, control.throttle,
                                                 control.brake]))
            self.manager.ego_vehicle.apply_control(control)

            self.world.tick()
            self.world.wait_for_tick()
            
            observation_dict = self.get_observation_dict()
            status = self.manager.scenario.test_criteria.status
            if status in [Status.SUCCESS, Status.FAILURE]:
                done = True
        else:
            done = True
        return observation_dict, done

    def reset(self):
        print("!!!reset env!!!")
        # Provide outputs if required
        self.analyze_scenario()

        # Stop scenario and cleanup
        self.manager.stop_scenario()
        self.cleanup(ego=True)
        if self.manager is not None:
            del self.manager
#        self.final_summary()
        self.start()
        return self.get_observation_dict()
