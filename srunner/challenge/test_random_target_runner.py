from unittest import TestCase
from srunner.challenge.random_target_runner import RandomTargetRunner


class TestRandomTargetRunner(TestCase):
    def setUp(self):
        self.runner = RandomTargetRunner(host="0.0.0.0", port="2000", num_vehicles=1, debug=True)
        print("done set up")
        
    def tearDown(self):
        print("start tear down")
        self.runner.reset()
        print("done tear down")

    def test_step(self):
        action = [0, 0, 0]
        print("action:")
        res = self.runner.step(action)
        print("done: ", res)
