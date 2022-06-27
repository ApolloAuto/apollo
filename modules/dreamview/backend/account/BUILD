load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "account_gflags",
    srcs = ["account_gflags.cc"],
    hdrs = ["account_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "account_updater",
    srcs = ["account_updater.cc"],
    hdrs = ["account_updater.h"],
    linkopts = ["-lcurl"],
    deps = [
        ":account_gflags",
        ":login_callback_handler",
        "//cyber",
        "//modules/common/util:json_util",
        "//modules/dreamview/backend/common:curl_request",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/dreamview/proto:account_info_cc_proto",
        "@boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_nlohmann_json//:json",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "login_callback_handler",
    srcs = ["login_callback_handler.cc"],
    hdrs = ["login_callback_handler.h"],
    deps = [
        "//cyber",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "@civetweb//:civetweb++",
    ],
)

cpplint()
