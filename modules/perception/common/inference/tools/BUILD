load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cal_table_generator",
    srcs = ["cal_table_generator.cc"],
    linkstatic = False,
    deps = [
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference/tensorrt:batch_stream",
        "//modules/perception/common/inference/tensorrt:entropy_calibrator",
        "//modules/perception/common/inference/tensorrt:rt_net",
        "//modules/perception/common/inference/utils:inference_util_lib",
    ],
)

cc_binary(
    name = "denseline_sample",
    srcs = ["denseline_sample.cc"],
    linkstatic = False,
    deps = [
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference/tensorrt:batch_stream",
        "//modules/perception/common/inference/tensorrt:entropy_calibrator",
        "//modules/perception/common/inference/tensorrt:rt_net",
        "//modules/perception/common/inference/utils:inference_util_lib",
    ],
)

cc_binary(
    name = "lane_sample",
    srcs = ["lane_sample.cc"],
    linkstatic = False,
    deps = [
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference/tensorrt:batch_stream",
        "//modules/perception/common/inference/tensorrt:entropy_calibrator",
        "//modules/perception/common/inference/tensorrt:rt_net",
        "//modules/perception/common/inference/utils:inference_util_lib",
    ],
)

cc_binary(
    name = "yolo_sample",
    srcs = ["yolo_sample.cc"],
    linkstatic = False,
    deps = [
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference/tensorrt:batch_stream",
        "//modules/perception/common/inference/tensorrt:entropy_calibrator",
        "//modules/perception/common/inference/tensorrt:rt_net",
        "//modules/perception/common/inference/utils:inference_util_lib",
    ],
)

apollo_package()
cpplint()
