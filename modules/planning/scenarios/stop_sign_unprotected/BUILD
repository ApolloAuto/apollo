load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stop_sign_unprotected_scenario",
    srcs = [
        "stop_sign_unprotected_scenario.cc",
        "stop_sign_unprotected_stage.cc",
    ],
    hdrs = [
        "stop_sign_unprotected_scenario.h",
        "stop_sign_unprotected_stage.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//external:gflags",
        "//cyber/common:log",
        "//modules/common",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/toolkits/deciders:decider",
        "@eigen",
    ],
)

cc_test(
    name = "stop_sign_unprotected_scenario_test",
    size = "small",
    srcs = ["stop_sign_unprotected_scenario_test.cc",],
    data = ["//modules/planning:planning_conf",],
    deps = [
        ":stop_sign_unprotected_scenario",
        "@gtest//:main",
    ],
)

cpplint()
