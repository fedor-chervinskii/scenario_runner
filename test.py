import os
import sys
import time
import carla
import multiprocessing as mp
from srunner.challenge.random_target_runner import RandomTargetRunner
from srunner.scenariomanager.scenario_manager import ScenarioManager
from catalyst.rl.environments.carla_wrapper import CarlaWrapper

os.environ["OMP_NUM_THREADS"] = "1"

def run():
    runner = RandomTargetRunner(host="0.0.0.0", port=2000, num_vehicles=2, debug=True)
    action = [0, 1, 0]  # throttling straight forward
    for i in range(200):
        print("action #{}:".format(i))
        res = runner.step(action)
        print("done: ", res)
        sys.stdout.flush()
    runner.reset()

def env_run():
    env = CarlaWrapper()
    env.step([0, 1, 0])
    env.reset()
    
#p = mp.Process(target=env_run)
#p.start()
#p.join()
if __name__ == "__main__":
    env = CarlaWrapper()