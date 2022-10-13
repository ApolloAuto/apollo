load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install", "install_files")

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

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":coord_transfrom",
    ],
    runtime_dest = "modules/v2x/tools/wgs84_utm_transform",
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "*.md",
    ]),
)

cpplint()
