import time
import sys
import os
import unittest

sys.path.append("../")
from cybertron import init

class TestInit(unittest.TestCase):
    def test_init(self):       
        self.assertTrue(init.init())
        self.assertTrue(init.ok())
        init.shutdown()
        self.assertTrue(init.is_shutdown())

if __name__ == '__main__':
  unittest.main()
  
