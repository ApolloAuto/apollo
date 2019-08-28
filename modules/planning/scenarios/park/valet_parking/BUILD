load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "valet_parking",
    srcs = [
        "stage_approaching_parking_spot.cc",
        "stage_parking.cc",
        "valet_parking_scenario.cc",
    ],
    hdrs = [
        "stage_approaching_parking_spot.h",
        "stage_parking.h",
        "valet_parking_scenario.h",
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
    name = "valet_parking_stages_test",
    size = "small",
    srcs = [
        "stage_approaching_parking_spot_test.cc",
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
