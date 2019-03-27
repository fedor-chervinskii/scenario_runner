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
import random
import time

import carla

from srunner.challenge.envs.sensor_interface import CallBack, Speedometer, HDMapReader, SensorInterface
from srunner.scenarios.challenge_basic import *
from srunner.scenarios.config_parser import *
from srunner.scenariomanager.scenario_manager import ScenarioManager

# Dictionary of supported scenarios.
# key = Name of config file in configs/
# value = List as defined in the scenario module
SCENARIOS = {
    "ChallengeBasic": CHALLENGE_BASIC_SCENARIOS
}


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
    client_timeout = 15.0  # in seconds
    wait_for_world = 10.0  # in seconds

    # CARLA world and scenario handlers
    world = None
    manager = None

    def __init__(self, args):
        self.output_scenario = []
        self._sensors_list = []
        self._hop_resolution = 2.0
        self.target = None
        self.spawn_point = None
        self.action_buffer = None
        self.sensor_interface = SensorInterface()
        self.start(args)

    def get_action(self):
        return self.action_buffer

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
        self.actors = []

        for i, _ in enumerate(self._sensors_list):
            if self._sensors_list[i] is not None:
                self._sensors_list[i].destroy()
                self._sensors_list[i] = None
        self._sensors_list = []

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

        for spawn_point in spawn_points:
            vehicle = self.world.try_spawn_actor(blueprint, spawn_point)
            if vehicle:
                self.spawn_point = spawn_point
                target = None
                for target in spawn_points:
                    if min_distance_to_goal < spawn_point.distance(target) < max_distance_to_goal:
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

    def prepare_actors(self, config):
        """
        Spawn or update all scenario actors according to
        their parameters provided in config
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
        for actor in config.other_actors:
            new_actor = self.setup_vehicle(actor.model, actor.transform, hero=False, autopilot=True,
                                           random_location=True)
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

    def start(self, args):
        """
        Run random target simulator
        """

        # Setup and run the scenarios for repetition times
        for _ in range(int(args.repetitions)):
            config = ScenarioConfiguration()
            config.town = "Town01"

            client = carla.Client(args.host, int(args.port))
            client.set_timeout(self.client_timeout)

            # Once we have a client we can retrieve the world that is currently
            # running.
            self.world = client.load_world(config.town)

            # Wait for the world to be ready
            self.world.wait_for_tick(self.wait_for_world)

            # Create scenario manager
            self.manager = ScenarioManager(self.world, args.debug)

            config.name = "RandomSpawnRandomTarget"
            config.type = "ChallengeBasic"
            for i in range(20):
                config.other_actors.append(DEFAULT_ACTOR)

            try:
                self.prepare_actors(config)
                pos_x, pos_y, pos_z = self.spawn_point.location.x, self.spawn_point.location.y, self.spawn_point.location.z
                config.ego_vehicle = ActorConfiguration(pos_x, pos_y, pos_z,
                                                        self.spawn_point.rotation.yaw,
                                                        False, False,
                                                        carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z)),
                                                        "vehicle.*")
                fin_x, fin_y, fin_z = self.target.location.x, self.target.location.y, self.target.location.z
                config.target = TargetConfiguration(fin_x, fin_y, fin_z,
                                                    carla.Transform(carla.Location(x=fin_x, y=fin_y, z=fin_z)))
                scenario = ChallengeBasic(self.world,
                                          self.ego_vehicle,
                                          self.actors,
                                          config.town,
                                          args.randomize,
                                          args.debug,
                                          config)
            except Exception as exception:
                print("The scenario cannot be loaded")
                print(exception)
                self.cleanup(ego=True)
                continue

            # Load scenario and run it
            self.manager.load_scenario(scenario)
            self.manager.start_scenario(self.get_action)
            self.success_criterion = InRadiusRegionTest(self.ego_vehicle,
                                                        x=self.target.transform.location.x,
                                                        y=self.target.transform.location.y,
                                                        radius=10.)

    def step(self, action):
        observation, reward, done, info = None, None, False, None
        if self.manager._running:
            control = carla.VehicleControl()
            control.steer = action[0]
            control.throttle = action[1]
            control.brake = action[2]
            control.hand_brake = False
            control.manual_gear_shift = False
            self.manager.ego_vehicle.apply_control(control)
#           input_data = self.sensor_interface.get_data()
            location = CarlaDataProvider.get_location(self.ego_vehicle)
            info = "location: {}".format(location)
            observation = [location.x, location.y, location.z]
            status = self.success_criterion.update()
            if status == "SUCCESS":
                reward = 1.
                done = True
            else:
                reward = -0.01
        else:
            done = True
        return observation, reward, done, info

    def reset(self):
        # Provide outputs if required
        self.analyze_scenario()

        # Stop scenario and cleanup
        self.manager.stop_scenario()

        self.cleanup(ego=True)

        self.final_summary()

