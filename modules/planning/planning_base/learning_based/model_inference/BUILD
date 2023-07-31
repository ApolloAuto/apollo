load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "model_inference",
    hdrs = ["model_inference.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/planning_base/proto:learning_data_cc_proto",
        "//modules/planning/planning_base/learning_based/model_inference/proto:learning_model_inference_cc_proto",
    ],
)

cc_library(
    name = "trajectory_imitation_libtorch_inference",
    srcs = ["trajectory_imitation_libtorch_inference.cc"],
    hdrs = ["trajectory_imitation_libtorch_inference.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":model_inference",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/planning_base/common/util:math_util_lib",
        "//modules/planning/planning_base/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        #"//third_party/libtorch",
        "@opencv//:core",
    ] + if_gpu(
        ["@libtorch_gpu"],
        ["@libtorch_cpu"],
    ),
)

cc_test(
    name = "model_inference_test",
    size = "medium",
    srcs = ["model_inference_test.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":model_inference",
        ":trajectory_imitation_libtorch_inference",
        "//modules/common/configs:config_gflags",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "@com_google_googletest//:gtest",
    ],
    linkstatic = True,
    tags = ["exclude"]
)

apollo_package()
cpplint()
