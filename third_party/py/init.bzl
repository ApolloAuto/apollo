load("//third_party/py:python_configure.bzl", "python_configure")
  
def init():
    python_configure(name = "local_config_python")