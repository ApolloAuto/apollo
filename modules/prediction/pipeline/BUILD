load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "records_to_offline_data",
    srcs = ["records_to_offline_data.cc"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:message_process",
        "//third_party:boost",
        "@com_google_absl//absl/strings",
        "@opencv",
    ],
)

cpplint()
