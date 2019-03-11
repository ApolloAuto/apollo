load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "valet_parking",
    srcs = [
        "valet_parking_scenario.cc",
        "stage_approaching_parking_spot.cc",
        "stage_parking.cc",
    ],
    hdrs = [
        "valet_parking_scenario.h",
        "stage_approaching_parking_spot.h",
        "stage_parking.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
    ],
)

cc_test(
    name = "valet_parking_scenario_test",
    size = "small",
    srcs = [
        "valet_parking_scenario_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":valet_parking",
        "@gtest//:main",
    ],
)

cc_test(
    name = "stage_approaching_parking_spot_test",
    size = "small",
    srcs = [
        "stage_approaching_parking_spot_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":valet_parking",
        "@gtest//:main",
    ],
)

cc_test(
    name = "stage_parking_test",
    size = "small",
    srcs = [
        "stage_parking_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":valet_parking",
        "@gtest//:main",
    ],
)

cpplint()
