load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "valet_parking",
    srcs = [
        "valet_parking_scenario.cc",
    ],
    hdrs = [
        "valet_parking_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":stage_approaching_parking_spot",
        ":stage_parking",
        "//cyber/common:log",
        "//modules/common",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
    ],
)

cc_library(
    name = "stage_approaching_parking_spot",
    srcs = [
        "stage_approaching_parking_spot.cc",
    ],
    hdrs = [
        "stage_approaching_parking_spot.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
    ],
)

cc_library(
    name = "stage_parking",
    srcs = [
        "stage_parking.cc",
    ],
    hdrs = [
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
        ":stage_approaching_parking_spot",
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
        ":stage_parking",
        "@gtest//:main",
    ],
)

cpplint()
