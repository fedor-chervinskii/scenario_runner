from unittest import TestCase
from srunner.challenge.random_target_runner import RandomTargetRunner


class TestRandomTargetRunner(TestCase):
    def setUp(self):
        self.runner = RandomTargetRunner(host="0.0.0.0", port="2000", debug=True)

    def test_start(self):
        self.runner.start()

    def test_step(self):
        action = [0, 0, 0]
        print(self.runner.step(action))

    def test_reset(self):
        self.runner.reset()