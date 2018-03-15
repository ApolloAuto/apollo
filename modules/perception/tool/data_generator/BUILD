load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_generator_lib",
    srcs = [
        "data_generator.cc",
        "velodyne64.cc",
    ],
    hdrs = [
        "data_generator.h",
        "sensor.h",
        "velodyne64.h",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/perception/common:pcl_util",
        "//modules/perception/tool/data_generator/common:data_generator_gflags",
        "//modules/perception/tool/data_generator/proto:config_proto",
    ],
)

cc_binary(
    name = "data_generator",
    srcs = ["main.cc"],
    deps = [
        ":data_generator_lib",
        "//external:gflags",
        "//modules/common:log",
        "@ros//:ros_common",
    ],
)

cpplint()
