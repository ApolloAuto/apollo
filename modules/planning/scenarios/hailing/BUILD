load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hailing",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":hailing_scenario",
    ],
)

cc_library(
    name = "hailing_scenario",
    srcs = [
        "hailing_scenario.cc",
    ],
    hdrs = [
        "hailing_scenario.h",
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
    name = "hailing_scenario_test",
    size = "small",
    srcs = [
        "hailing_scenario_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":hailing",
        "@gtest//:main",
    ],
)

cpplint()
