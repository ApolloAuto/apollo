load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "model_inference",
    hdrs = ["model_inference.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/proto:learning_data_proto",
        "//modules/planning/proto:task_config_proto",
    ],
)

cc_library(
    name = "trajectory_imitation_libtorch_inference",
    srcs = ["trajectory_imitation_libtorch_inference.cc"],
    hdrs = ["trajectory_imitation_libtorch_inference.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":model_inference",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common/util:math_util_lib",
        "//modules/planning/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "//third_party:libtorch",
        "@opencv",
    ],
)

cc_library(
    name = "trajectory_imitation_tensorrt_inference",
    srcs = ["trajectory_imitation_tensorrt_inference.cc"],
    hdrs = ["trajectory_imitation_tensorrt_inference.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":model_inference",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common/util:math_util_lib",
        "//modules/planning/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "//third_party:libtorch",
        "@cuda",
        "@opencv",
        "@tensorrt",
    ],
)

cc_test(
    name = "model_inference_test",
    size = "medium",
    srcs = ["model_inference_test.cc"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":model_inference",
        ":trajectory_imitation_libtorch_inference",
        ":trajectory_imitation_tensorrt_inference",
        "//modules/common/configs:config_gflags",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
