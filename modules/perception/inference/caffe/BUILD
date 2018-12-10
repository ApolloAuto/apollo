load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "caffe_net_lib",
    srcs = ["caffe_net.cc"],
    hdrs = ["caffe_net.h"],
    linkopts = ["-lopencv_core -lnvinfer_plugin -lboost_system -lopencv_imgproc -lopencv_highgui"],
    deps = [
        "//modules/perception/inference:inference_lib",
        "@caffe",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "caffe_net_test",
    size = "small",
    srcs = ["caffe_net_test.cc"],
    deps = [
        ":caffe_net_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "@gtest//:main",
    ],
)

cpplint()
