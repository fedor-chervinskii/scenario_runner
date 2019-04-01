import sys
import time
import carla
from srunner.challenge.random_target_runner import RandomTargetRunner
from srunner.scenariomanager.scenario_manager import ScenarioManager

if True:
    runner = RandomTargetRunner(host="0.0.0.0", port="2000", num_vehicles=1, debug=True)
    action = [0, 1, 0]  # throttling straight forward
    for i in range(200):
        print("action #{}:".format(i))
        res = runner.step(action)
        print("done: ", res)
        sys.stdout.flush()
    runner.reset()
