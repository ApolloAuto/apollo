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

cpplint()
