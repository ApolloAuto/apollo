load("//third_party/tensorrt:tensorrt_configure.bzl", "tensorrt_configure")
  
def init():
    tensorrt_configure(name = "local_config_tensorrt")