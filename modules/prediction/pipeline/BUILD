load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
        "@com_google_absl//absl/strings",
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
        "//cyber/common:file",
        "//modules/common/math:linear_interpolation",
        "//modules/common/util:point_factory",
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

cpplint()
