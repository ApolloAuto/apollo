load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "cal_table_generator",
    srcs = ["cal_table_generator.cc"],
    deps = [
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference/tensorrt:apollo_perception_inference_rt_common",
        "//modules/perception/common/inference/tensorrt:apollo_perception_inference_rt_net",
        "//modules/perception/common/inference/utils:apollo_perception_common_utils",
    ],
)

apollo_cc_binary(
    name = "denseline_sample",
    srcs = ["denseline_sample.cc"],
    deps = [
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference/tensorrt:apollo_perception_inference_rt_common",
        "//modules/perception/common/inference/tensorrt:apollo_perception_inference_rt_net",
        "//modules/perception/common/inference/utils:apollo_perception_common_utils",
    ],
)

apollo_cc_binary(
    name = "lane_sample",
    srcs = ["lane_sample.cc"],
    deps = [
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference/tensorrt:apollo_perception_inference_rt_common",
        "//modules/perception/common/inference/tensorrt:apollo_perception_inference_rt_net",
        "//modules/perception/common/inference/utils:apollo_perception_common_utils",
    ],
)

apollo_cc_binary(
    name = "yolo_sample",
    srcs = ["yolo_sample.cc"],
    deps = [
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference/tensorrt:apollo_perception_inference_rt_common",
        "//modules/perception/common/inference/tensorrt:apollo_perception_inference_rt_net",
        "//modules/perception/common/inference/utils:apollo_perception_common_utils",
    ],
)

apollo_package()
cpplint()
