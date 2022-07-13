load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

V2X_COPTS = ["-DMODULE_NAME=\\\"v2x\\\""]

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "v2x_proxy_gflags",
    srcs = ["v2x_proxy_gflags.cc"],
    hdrs = ["v2x_proxy_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "json_parse",
    hdrs = ["json_parse.h"],
    copts = V2X_COPTS,
    deps = [
        "//cyber/common:log",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "json_parse_test",
    size = "small",
    srcs = ["json_parse_test.cc"],
    copts = V2X_COPTS,
    deps = [
        ":json_parse",
        "//cyber",
        "//modules/v2x/proto:spat_data_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
