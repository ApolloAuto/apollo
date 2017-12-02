load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_base",
    srcs = [
        "image.cc",
        "light.cc",
        "pose.cc",
        "tl_shared_data.cc",
        "utils.cc",
    ],
    hdrs = [
        "image.h",
        "image_lights.h",
        "light.h",
        "pose.h",
        "tl_shared_data.h",
        "utils.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/map/proto:map_proto",
        "//modules/perception/lib/config_manager",
        "//modules/perception/onboard:perception_onboard",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/traffic_light/util:perception_traffic_light_util",
    ],
)

cpplint()
