load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "configurator",
    srcs = ["configurator.cc"],
    hdrs = ["configurator.h"],
    deps = [
        "//cyber",
        "//modules/common/util:json_util",
        "//modules/dreamview/backend/account:account_gflags",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/common:curl_request",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/proto:account_info_cc_proto",
        "//modules/dreamview/proto:configuration_profile_status_cc_proto",
        "@boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
    ],
)

cpplint()
