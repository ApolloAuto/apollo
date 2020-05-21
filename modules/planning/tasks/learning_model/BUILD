load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "learning_model_inference_task",
    srcs = ["learning_model_inference_task.cc"],
    hdrs = ["learning_model_inference_task.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks:task",
        "//third_party:libtorch",
    ],
)

cc_test(
    name = "learning_model_inference_task_test",
    size = "small",
    srcs = ["learning_model_inference_task_test.cc"],
    deps = [
        "learning_model_inference_task",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
