load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "integration_test_base",
    srcs = [
        "integration_test_base.cc",
    ],
    hdrs = [
        "integration_test_base.h",
    ],
    data = [
        "//modules/map:map_data",
        "//modules/planning:planning_conf",
        "//modules/planning:planning_testdata",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning:planning_component_lib",
        "@gtest",
    ],
)

cc_test(
    name = "sunnyvale_big_loop_test",
    size = "small",
    srcs = [
        "sunnyvale_big_loop_test.cc",
    ],
    data = [
        "//modules/common/configs:config_gflags",
        "//modules/map:map_data",
        "//modules/planning:planning_testdata",
    ],
    deps = [
        ":integration_test_base",
    ],
)

cpplint()
