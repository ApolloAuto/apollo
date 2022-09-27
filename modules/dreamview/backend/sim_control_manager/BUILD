load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sim_control_manager",
    srcs = ["sim_control_manager.cc"],
    hdrs = [
        "sim_control_manager.h",
    ],
    alwayslink = True,
    copts = ['-DMODULE_NAME=\\"dreamview\\"'],
    deps = [
        "//modules/dreamview/backend/sim_control_manager/common:sim_control_gflags",
        "//cyber",
        "//modules/dreamview/backend/sim_control_manager/core:dynamic_model_factory",
        "//modules/dreamview/backend/sim_control_manager/core:sim_control_base",
        "//modules/common/configs:config_gflags",
        "@com_github_nlohmann_json//:json",
    ],
)

# todo: add test file
# cc_test(
#     name = "sim_control_test",
#     size = "small",
#     srcs = ["sim_control_test.cc"],
#     data = [
#         "//modules/dreamview/backend/testdata",
#     ],
#     deps = [
#         ":sim_control",
#         "//cyber",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cpplint()
