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
    srcs = [
        "stage_run.cc",
        "test_learning_model_scenario.cc",
    ],
    hdrs = [
        "stage_run.h",
        "test_learning_model_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/util:factory",
        "//modules/planning/reference_line",
        "//modules/planning/scenarios:scenario",
        "//third_party:libtorch",
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
