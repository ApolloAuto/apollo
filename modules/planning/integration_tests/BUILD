load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planning_test_base",
    srcs = [
        "planning_test_base.cc",
    ],
    hdrs = [
        "planning_test_base.h",
    ],
    data = [
        "//modules/planning:planning_conf",
        "//modules/planning:planning_testdata",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning:planning_component_lib",
        "//modules/planning:planning_lib",
        "//modules/planning/proto:dp_st_speed_config_proto",
        "@gtest",
    ],
)

# FIXME(all): temporarily disable integration test for planning flaky problems.

# cc_test(
#     name = "garage_test",
#     size = "small",
#     srcs = [
#         "garage_test.cc",
#     ],
#     data = [
#         "//modules/common/configs:config_gflags",
#         "//modules/planning:planning_testdata",
#     ],
#     deps = [
#         ":planning_test_base",
#     ],
# )

# cc_test(
#     name = "sunnyvale_loop_test",
#     size = "small",
#     srcs = [
#         "sunnyvale_loop_test.cc",
#     ],
#     data = [
#         "//modules/common/configs:config_gflags",
#         "//modules/map/data:map_sunnyvale_loop",
#         "//modules/planning:planning_testdata",
#     ],
#     deps = [
#         ":planning_test_base",
#     ],
# )

# cc_test(
#     name = "sunnyvale_big_loop_test",
#     size = "small",
#     srcs = [
#         "sunnyvale_big_loop_test.cc",
#     ],
#     data = [
#         "//modules/common/configs:config_gflags",
#         "//modules/map/data:map_sunnyvale_big_loop",
#         "//modules/planning:planning_testdata",
#     ],
#     deps = [
#         ":planning_test_base",
#     ],
# )

# cc_test(
#     name = "navigation_mode_test",
#     size = "small",
#     srcs = [
#         "navigation_mode_test.cc",
#     ],
#     data = [
#         "//modules/common/configs:config_gflags",
#         "//modules/planning:planning_testdata",
#     ],
#     deps = [
#         ":planning_test_base",
#     ],
# )

cpplint()
