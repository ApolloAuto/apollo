load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "caffe_net_lib",
    srcs = ["caffe_net.cc"],
    hdrs = ["caffe_net.h"],
    linkopts = ["-lnvinfer_plugin"],
    deps = [
        "//modules/perception/inference:inference_lib",
        "//third_party:boost",
        "@caffe",
        "@com_google_protobuf//:protobuf",
        "@opencv",
    ],
)

cc_test(
    name = "caffe_net_test",
    size = "small",
    srcs = ["caffe_net_test.cc"],
    deps = [
        ":caffe_net_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
