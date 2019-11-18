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
    linkopts = select(
        {
            ":x86_mode": [
                "-L/usr/lib/x86_64-linux-gnu/",
            ],
            ":arm_mode": [
                "-L/usr/lib/aarch64-linux-gnu/",
            ],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ) + [
        "-lnvcaffe_parser",
        "-lnvinfer",
        "-lnvinfer_plugin",
        "-lnvparsers",
    ],
    visibility = ["//visibility:public"],
)

config_setting(
    name = "x86_mode",
    values = {"cpu": "k8"},
)

config_setting(
    name = "arm_mode",
    values = {"cpu": "arm"},
)
