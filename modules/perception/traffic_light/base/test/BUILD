load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_traffic_light_base_test",
    srcs = [
        "image_test.cc",
        "light_test.cc",
        "utils_structs_test.cc",
    ],
    hdrs = [
    ],
    deps = [
        "//modules/common:log",
        "//modules/map/proto:map_proto",
        "//modules/perception/lib/config_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/traffic_light/base:perception_traffic_light_base",
        "//modules/perception/traffic_light/util:perception_traffic_light_util",
    ],
)

cpplint()
