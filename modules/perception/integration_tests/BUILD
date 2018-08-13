load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_test_base",
    srcs = [
        "perception_test_base.cc",
    ],
    hdrs = [
        "perception_test_base.h",
    ],
    data = [
        "//modules/map:map_data",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/perception:perception_lib",
        "@gtest",
    ],
)

cpplint()
