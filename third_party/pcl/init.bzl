load("//third_party/pcl:pcl_configure.bzl", "pcl_configure")
  
def init():
    pcl_configure(name = "local_config_pcl")
