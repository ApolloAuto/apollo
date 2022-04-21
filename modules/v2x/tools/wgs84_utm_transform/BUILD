load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "coord_transfrom",
    srcs = ["coord_transfrom.cc"],
    deps = [
        "//cyber",
        "//modules/map/hdmap/adapter:coordinate_convert_tool",
        "@com_google_absl//absl/strings",
    ],
)

cpplint()
