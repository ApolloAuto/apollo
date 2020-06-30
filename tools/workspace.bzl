# Apollo external dependencies that can be loaded in WORKSPACE files.
load("//third_party/adolc:workspace.bzl", adolc = "repo")
load("//third_party/adv_plat:workspace.bzl", adv_plat = "repo")
load("//third_party/fastrtps:workspace.bzl", fastrtps = "repo")
load("//third_party/gflags:workspace.bzl", gflags = "repo")
load("//third_party/glog:workspace.bzl", glog = "repo")
load("//third_party/eigen3:workspace.bzl", eigen = "repo")
load("//third_party/boost:workspace.bzl", boost = "repo")

def initialize_third_party():
    """ Load third party repositories.  See above load() statements. """
    adolc()
    adv_plat()
    gflags()
    glog()
    eigen()
    fastrtps()
    boost()
    

# Define all external repositories required by
def apollo_repositories():
    initialize_third_party()
