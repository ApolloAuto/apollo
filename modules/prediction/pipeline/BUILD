load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "records_to_offline_data",
    srcs = ["records_to_offline_data.cc"],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        "//modules/prediction/common:message_process",
    ],
)

cpplint()
