load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "prediction/bin",
    targets = [
        ":records_to_offline_data",
        ":vector_net_feature",
        ":vector_net_offline_data",
        ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "records_to_offline_data",
    srcs = ["records_to_offline_data.cc"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    linkopts = [
        "-lgomp",
    ],
    deps = [
        "//modules/prediction/common:message_process",
        "@boost",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "vector_net",
    srcs = ["vector_net.cc"],
    hdrs = ["vector_net.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/map/hdmap:hdmap_util",
        "//modules/prediction/common:prediction_system_gflags",
        "//modules/prediction/proto:vector_net_cc_proto",
    ],
)

cc_binary(
    name = "vector_net_feature",
    srcs = ["vector_net_feature.cc"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":vector_net",
    ],
)

cc_binary(
    name = "vector_net_offline_data",
    srcs = ["vector_net_offline_data.cc"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":vector_net",
    ],
)

cpplint()
