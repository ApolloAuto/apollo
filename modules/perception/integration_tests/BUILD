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

filegroup(
    name = "integration_tests_data",
    srcs = glob([
        "testdata/**/*",
    ]),
)

cc_test(
    name = "sunnyvale_big_loop_test",
    size = "small",
    srcs = [
        "sunnyvale_big_loop_test.cc",
    ],
    data = [
        ":integration_tests_data",
        "//modules/common/configs:config_gflags",
        "//modules/map:map_data",
    ],
    deps = [
        ":perception_test_base",
    ],
)

cpplint()
