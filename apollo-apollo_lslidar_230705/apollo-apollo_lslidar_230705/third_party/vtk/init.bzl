load("//third_party/vtk:vtk_configure.bzl", "vtk_configure")
  
def init():
    vtk_configure(name = "local_config_vtk")