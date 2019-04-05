#!/usr/bin/env python

#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Basic CARLA Autonomous Driving training scenario
"""

import py_trees

from srunner.scenariomanager.atomic_scenario_behavior import *
from srunner.scenariomanager.atomic_scenario_criteria import *
from srunner.scenarios.basic_scenario import *
from srunner.scenariomanager.timer import TimeOut


CHALLENGE_BASIC_SCENARIOS = ["ChallengeBasic"]


class ChallengeVeryBasic(BasicScenario):

    """
    Implementation of a dummy scenario
    """

    category = "ChallengeBasic"
    radius = 10.0           # meters

    def __init__(self, world, ego_vehicle, other_actors, town, randomize=False, debug_mode=False, config=None, timeout=300):
        """
        Setup all relevant parameters and create scenario
        """
        self.config = config
        self.target = None
        self.route = None
        self.timeout = timeout

        if hasattr(self.config, 'target'):
            self.target = self.config.target

        super(ChallengeVeryBasic, self).__init__("ChallengeVeryBasic", ego_vehicle, other_actors, town, world, debug_mode, True)

    def _create_behavior(self):
        """
        Basic behavior do nothing, i.e. Idle
        """

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        idle_behavior = Idle()
        sequence.add_child(idle_behavior)

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """

        collision_criterion = CollisionTest(self.ego_vehicle, terminate_on_failure=True)
        target_criterion = InRadiusRegionTest(self.ego_vehicle,
                                             x=self.target.transform.location.x ,
                                             y=self.target.transform.location.y,
                                             radius=self.radius)

        wrong_way_criterion = WrongLaneTest(self.ego_vehicle)

        red_light_criterion = RunningRedLightTest(self.ego_vehicle)

        parallel_criteria = py_trees.composites.Parallel("group_criteria",
                                                         policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        timeout = TimeOut(self.timeout)
        
        parallel_criteria.add_child(collision_criterion)
        parallel_criteria.add_child(target_criterion)
        parallel_criteria.add_child(wrong_way_criterion)
        parallel_criteria.add_child(red_light_criterion)
        parallel_criteria.add_child(timeout)

        return parallel_criteria
