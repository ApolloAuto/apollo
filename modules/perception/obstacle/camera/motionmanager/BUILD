load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_camera_motionmanager",
    srcs = [
        "vehicleplanemotion.cc"
    ],
    hdrs = [
        "vehicleplanemotion.h",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "//modules/perception/obstacle/base:perception_obstacle_base",
    ],
)

cpplint()