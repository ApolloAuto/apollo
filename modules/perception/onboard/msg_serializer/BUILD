load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "msg_serializer",
    srcs = ["msg_serializer.cc"],
    hdrs = ["msg_serializer.h"],
    deps = [
        "//cyber",
        "//modules/common/time",
        "//modules/perception/base",
        "//modules/perception/lib/utils",
        "//modules/perception/onboard/common_flags",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/proto:feature_proto",
        "@eigen",
    ],
)

cpplint()
