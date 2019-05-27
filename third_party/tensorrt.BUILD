# Eigen is a C++ template library for linear algebra: vectors,
# matrices, and related algorithms.
#
# This is Eigen 3.2.10

# This BUILD file is derived from
# https://github.com/RobotLocomotion/drake/blob/master/tools/eigen.BUILD

licenses([
    # Note: Eigen is an MPL2 library that includes GPL v3 and LGPL v2.1+ code.
    #       We've taken special care to not reference any restricted code.
    "notice",  # Portions BSD
])

cc_library(
    name = "tensorrt",
    includes = [
        ".",
        "/usr/include/tensorrt",
    ],
    linkopts = [
        "-L/usr/lib/x86_64-linux-gnu/",
        "-lnvcaffe_parser",
        "-lnvinfer",
        "-lnvinfer_plugin",
        "-lnvparsers",
    ],
    visibility = ["//visibility:public"],
)
