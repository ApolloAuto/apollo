import ctypes
from ctypes import cdll, c_ushort
from ctypes import c_double
from ctypes import POINTER
import math

lib = cdll.LoadLibrary('/apollo/bazel-bin/modules/planning/planner/open_space/hybrid_a_star_wrapper_lib.so')

class HybridAStarPlanner(object):
    def __init__(self):
        self.planner = lib.CreatePlannerPtr()
        self.obstacles = lib.CreateObstaclesPtr()
        self.result = lib.CreateResultPtr()
    def AddVirtualObstacle(self, x, y, heading, length, width):
        lib.AddVirtualObstacle(self.obstacles, c_double(x), 
            c_double(y), c_double(heading), c_double(length), c_double(width))
    def Plan(self, sx, sy, sphi, ex, ey, ephi):
        return lib.Plan(self.planner, self.obstacles, self.result, c_double(sx), 
            c_double(sy), c_double(sphi), c_double(ex), c_double(ey),c_double(ephi))
    def GetResult(self, x, y, phi, output_size) :
        lib.GetResult(self.result, POINTER(c_double)(x), POINTER(c_double)(y), 
            POINTER(c_double)(phi), POINTER(c_ushort)(output_size))