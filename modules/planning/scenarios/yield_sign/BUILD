load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "yield_sign_scenario",
    srcs = [
        "yield_sign_scenario.cc",
    ],
    hdrs = [
        "yield_sign_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios/util:scenario_util_lib",
        "@eigen",
    ],
)

#cc_test(
#    name = "yield_sign_scenario_test",
#    size = "small",
#    srcs = [
#        "yield_sign_scenario_test.cc",
#    ],
#    data = [
#        "//modules/planning:planning_conf",
#    ],
#    deps = [
#        ":yield_sign_scenario",
#        "@gtest//:main",
#    ],
#)

cpplint()
