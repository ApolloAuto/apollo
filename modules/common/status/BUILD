load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "status",
    hdrs = ["status.h"],
    deps = [
        "//modules/common/proto:error_code_proto",
        "//modules/common/util:future",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "status_test",
    size = "small",
    srcs = ["status_test.cc"],
    deps = [
        ":status",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
