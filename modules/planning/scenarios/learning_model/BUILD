load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "learning_model",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":test_learning_model_scenario",
    ],
)

cc_library(
    name = "test_learning_model_scenario",
    srcs = ["test_learning_model_scenario.cc"],
    hdrs = ["test_learning_model_scenario.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/constraint_checker",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/scenarios:scenario",
        "//third_party:libtorch",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cc_test(
    name = "test_learning_model_scenario_test",
    size = "small",
    srcs = ["test_learning_model_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":test_learning_model_scenario",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
