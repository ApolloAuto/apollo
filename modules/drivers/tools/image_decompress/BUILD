load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "drivers/lib/tools/image_decompress",
    data_dest = "drivers/addition_data/tools/image_decompress",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libimage_decompress.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_library(
    name = "image_decompress_lib",
    srcs = ["image_decompress.cc"],
    hdrs = ["image_decompress.h"],
    copts = ['-DMODULE_NAME=\\"image_decompress\\"'],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/drivers/tools/image_decompress/proto:config_cc_proto",
        "@opencv//:imgcodecs",
    ],
)

cc_test(
    name = "image_decompress_test",
    size = "small",
    srcs = ["image_decompress_test.cc"],
    deps = [
        ":image_decompress_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "libimage_decompress.so",
    linkshared = True,
    linkstatic = True,
    deps = [":image_decompress_lib"],
)

cpplint()
