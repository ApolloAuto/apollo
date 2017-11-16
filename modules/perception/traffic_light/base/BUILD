load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_base",
    srcs = [
        "image.cc",
        "light.cc",
        "pose.cc",
        "utils.cc",
    ],
    hdrs = [
        "image_lights.h",
        "light.h",
        "image.h",
        "pose.h",
        "utils.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/config_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/map/proto:map_proto"
    ],
    copts=["-mavx2"],
)

cpplint()