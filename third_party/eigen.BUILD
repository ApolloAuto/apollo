# Eigen is a C++ template library for linear algebra: vectors,
# matrices, and related algorithms.
#
# This is Eigen 3.2.10

# This BUILD file is derived from
# https://github.com/RobotLocomotion/drake/blob/master/tools/eigen.BUILD

licenses([
    # Note: Eigen is an MPL2 library that includes GPL v3 and LGPL v2.1+ code.
    #       We've taken special care to not reference any restricted code.
    "reciprocal",  # MPL2
    "notice",  # Portions BSD
])

exports_files(["COPYING.MPL2"])

cc_library(
    name = "eigen",
    hdrs = glob([
        "Eigen/*",
        "Eigen/**/*.h",
        "unsupported/Eigen/*",
        "unsupported/Eigen/**/*.h",
    ]),
    defines = ["EIGEN_MPL2_ONLY"],
    includes = ["."],
    visibility = ["//visibility:public"],
)
