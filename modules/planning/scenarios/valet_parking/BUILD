load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "valet_parking",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":valet_parking_scenario",
    ],
)

cc_library(
    name = "valet_parking_scenario",
    srcs = [
        "valet_parking_scenario.cc",
    ],
    hdrs = [
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

cpplint()
