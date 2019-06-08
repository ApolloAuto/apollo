load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "paddle_net_lib",
    srcs = ["paddle_net.cc"],
    hdrs = ["paddle_net.h"],
    linkopts = ["-lglog",
                "-lsnappy",
                "-lboost_system",
                "-lboost_filesystem",
                "-lboost_thread",
                "-lboost_program_options",
                "-lz",
                "-lsnappy",
                "-lrt",
                "-ldl",
                "-lpthread",
                ],
    deps = [
        "@paddlepaddle",
        "@com_google_protobuf//:protobuf",
        "//modules/perception/inference:inference_lib",
        "//external:gflags",
        "@xxhash",
        "@snappystream",
        "@eigen",
        "@mklml",
        "@mkldnn",
        "@cuda",
    ],
    linkstatic=False,
)

#cc_test(
#    name = "caffe_net_test",
#    size = "small",
#    srcs = ["caffe_net_test.cc"],
#    deps = [
#        ":caffe_net_lib",
#        "//modules/perception/inference/utils:inference_util_lib",
#        "@gtest//:main",
#    ],
#)

cpplint()
