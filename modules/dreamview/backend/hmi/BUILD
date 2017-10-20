load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hmi_status_helper",
    srcs = ["hmi_status_helper.cc"],
    hdrs = ["hmi_status_helper.h"],
    deps = [
        "//modules/dreamview/proto:hmi_status_proto",
    ],
)

cpplint()
