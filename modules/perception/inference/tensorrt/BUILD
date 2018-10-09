load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rt_common",
    srcs = [
        "rt_common.cc",
    ],
    hdrs = [
        "rt_common.h",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/perception/base:common",
        "//modules/perception/proto:rt_proto",
        "@tensorrt",
    ],
)

cc_test(
    name = "rt_common_test",
    size = "small",
    srcs = ["rt_common_test.cc"],
    deps = [
        ":rt_common",
        "@gtest//:main",
    ],
)

cc_library(
    name = "rt_utils",
    srcs = [
        "rt_utils.cc",
    ],
    hdrs = [
        "rt_utils.h",
    ],
    deps = [
        "//modules/perception/proto:rt_proto",
        "//framework:cybertron",
        "@tensorrt",
    ],
)

cc_test(
    name = "rt_utils_test",
    size = "small",
    srcs = ["rt_utils_test.cc"],
    deps = [
        ":rt_utils",
        "@gtest//:main",
    ],
)

cc_library(
    name = "rt_net",
    srcs = [
        "rt_net.cc",
    ],
    hdrs = [
        "rt_net.h",
        "entropy_calibrator.h",
    ],
    linkopts = ["-lopencv_core -lnvinfer_plugin -lopencv_imgproc -lopencv_highgui -lprotobuf -lcaffe"],
    deps = [
        ":rt_common",
        ":rt_utils",
        ":batch_stream",
        ":entropy_calibrator",
        "//modules/perception/base:base",
        "//modules/perception/proto:rt_proto",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/tensorrt/plugins:perception_inference_tensorrt_plugins",
        "//framework:cybertron",
        "@tensorrt",
    ],
)

cc_test(
    name = "rt_net_test",
    size = "small",
    srcs = ["rt_net_test.cc"],
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    deps = [
        ":rt_net",
        "//modules/perception/inference:inference_factory_lib",
        "@gtest//:main",
    ],
)

cc_library(
    name = "entropy_calibrator",
    hdrs = [
        "entropy_calibrator.h",
    ],
    linkopts = ["-lopencv_core -lnvinfer_plugin -lopencv_imgproc -lopencv_highgui -lprotobuf -lcaffe"],
    deps = [
        ":rt_utils",
        ":batch_stream",
        "@tensorrt",
        "@cuda",
    ],
)

cc_test(
    name = "entropy_calibrator_test",
    size = "small",
    srcs = ["entropy_calibrator_test.cc"],
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    deps = [
        ":entropy_calibrator",
        "@gtest//:main",
    ],
)

cc_library(
    name = "batch_stream",
    srcs = [
        "batch_stream.cc",
    ],
    hdrs = [
        "batch_stream.h",
    ],
    linkopts = ["-lopencv_core -lnvinfer_plugin -lopencv_imgproc -lopencv_highgui -lprotobuf -lcaffe"],
    deps = [
        "//modules/perception/proto:rt_proto",
        "//framework:cybertron",
        "@tensorrt",
    ],
)

cc_test(
    name = "batch_stream_test",
    size = "small",
    srcs = ["batch_stream_test.cc"],
    data = [
        "//modules/perception/inference:inference_test_data",
    ],
    deps = [
        ":batch_stream",
        "@gtest//:main",
    ],
)

cpplint()
