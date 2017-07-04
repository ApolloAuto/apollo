load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hmi_status_helper",
    srcs = ["hmi_status_helper.cc"],
    hdrs = ["hmi_status_helper.h"],
    deps = [
        ":restful_client",
        "//external:gflags",
        "//modules/hmi/proto:runtime_status_proto",
    ],
)

cc_library(
    name = "restful_client",
    srcs = ["restful_client.cc"],
    hdrs = ["restful_client.h"],
    deps = [
        "//modules/common:log",
        "@com_github_google_protobuf//:protobuf",
        "@curlpp//:curlpp",
        "@glog//:glog",
    ],
)

cc_test(
    name = "restful_client_test",
    size = "small",
    srcs = ["restful_client_test.cc"],
    deps = ["//modules/hmi/utils:restful_client",
            "//modules/hmi/proto:runtime_status_proto",
            "@gtest//:main",
            "//external:gflags",
    ]
)

cpplint()
