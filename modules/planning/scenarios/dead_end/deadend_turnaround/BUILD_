load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "deadend_turnaround",
    srcs = [
        "stage_approaching_turning_point.cc",
        "stage_turning.cc",
        "deadend_turnaround_scenario.cc",
    ],
    hdrs = [
        "stage_approaching_turning_point.h",
        "stage_turning.h",
        "deadend_turnaround_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/planning/scenarios:scenario",
    ],
)

cc_test(
    name = "deadend_turnaround_scenario_test",
    size = "small",
    srcs = ["deadend_turnaround_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":deadend_turnaround",
        "@com_google_googletest//:gtest_main",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
    linkstatic = True,
)

cc_test(
    name = "stage_approaching_turning_point_test",
    size = "small",
    srcs = ["stage_approaching_turning_point_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":deadend_turnaround",
        "@com_google_googletest//:gtest_main",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
    linkstatic = True,
)

cc_test(
    name = "stage_turning_test",
    size = "small",
    srcs = ["stage_turning_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    linkopts = ["-lgomp"],
    deps = [
        ":deadend_turnaround",
        "@com_google_googletest//:gtest_main",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
    linkstatic = True,
)

cpplint()
