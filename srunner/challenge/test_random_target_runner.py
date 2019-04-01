from unittest import TestCase
from srunner.challenge.random_target_runner import RandomTargetRunner
import sys


class TestRandomTargetRunner(TestCase):
    def setUp(self):
        self.runner = RandomTargetRunner(host="0.0.0.0", port="2000", num_vehicles=1, debug=True)
        print("done set up")

    def tearDown(self):
        del self.runner
        
    def test_step(self):
        action = [0, 1, 0]
        print("action:")
        res = self.runner.step(action)
        print("done: ", res)
        
    def test_hundred_steps(self):
        action = [0, 1, 0]
        for i in range(100):
            print("action #{}:".format(i))
            res = self.runner.step(action)
            print("done: ", res)

    def test_reset(self):
        self.runner.reset()