load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "paddle_net_lib",
    srcs = ["paddle_net.cc"],
    hdrs = ["paddle_net.h"],
    linkopts = [
        "-lsnappy",
        "-lz",
        "-ldl",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/perception/inference:inference_lib",
        "//third_party:boost",
        "//third_party:pthread",
        "//third_party:rt",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_protobuf//:protobuf",
        "@cuda",
        "@eigen",
        "@mklml",
        "@paddlepaddle",
        "@snappystream",
        "@xxhash",
    ],
)

cpplint()
