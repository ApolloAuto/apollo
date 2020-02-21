load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cal_table_generator",
    srcs = ["cal_table_generator.cc"],
    linkstatic = False,
    deps = [
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/tensorrt:batch_stream",
        "//modules/perception/inference/tensorrt:entropy_calibrator",
        "//modules/perception/inference/tensorrt:rt_net",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_binary(
    name = "denseline_sample",
    srcs = ["denseline_sample.cc"],
    linkstatic = False,
    deps = [
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/tensorrt:batch_stream",
        "//modules/perception/inference/tensorrt:entropy_calibrator",
        "//modules/perception/inference/tensorrt:rt_net",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_binary(
    name = "lane_sample",
    srcs = ["lane_sample.cc"],
    linkstatic = False,
    deps = [
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/tensorrt:batch_stream",
        "//modules/perception/inference/tensorrt:entropy_calibrator",
        "//modules/perception/inference/tensorrt:rt_net",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_binary(
    name = "yolo_sample",
    srcs = ["yolo_sample.cc"],
    linkstatic = False,
    deps = [
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/tensorrt:batch_stream",
        "//modules/perception/inference/tensorrt:entropy_calibrator",
        "//modules/perception/inference/tensorrt:rt_net",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cpplint()
