load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "l3_perception_lib",
    srcs = ["l3_perception.cc"],
    hdrs = ["l3_perception.h"],
    deps = [
        ":l3_perception_gflags",
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/proto:localization_proto"
    ],
)

cc_binary(
    name = "l3_perception",
    srcs = ["main.cc"],
    linkstatic = 0,
    deps = [
        ":l3_perception_lib",
        "//external:gflags",
        "//modules/common:log",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "l3_perception_gflags",
    srcs = [
        "l3_perception_gflags.cc",
    ],
    hdrs = [
        "l3_perception_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cpplint()
