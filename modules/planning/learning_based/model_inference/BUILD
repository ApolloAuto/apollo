load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "model_inference",
    hdrs = ["model_inference.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/proto:learning_data_proto"
    ],
)

cc_library(
    name = "trajectory_imitation_inference",
    hdrs = ["trajectory_imitation_inference.h"],
    srcs = ["trajectory_imitation_inference.cc"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":model_inference",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/learning_based/img_feature_renderer:birdview_img_feature_renderer",
        "//third_party:libtorch",
        "@opencv",
    ],
)

cpplint()
