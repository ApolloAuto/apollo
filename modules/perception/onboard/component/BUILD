load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lidar_output_component",
    srcs = [
        "lidar_output_component.cc",
    ],
    hdrs = [
        "lidar_output_component.h",
    ],
    deps = [
        "@eigen",
        "//framework:cybertron",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/base",
        "//modules/perception/onboard/inner_component_messages",
        "//modules/perception/onboard/msg_serializer",
    ],
)

cpplint()
