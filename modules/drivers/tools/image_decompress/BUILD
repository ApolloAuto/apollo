load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "image_decompress_lib",
    srcs = ["image_decompress.cc"],
    hdrs = ["image_decompress.h"],
    copts = ['-DMODULE_NAME=\\"image_decompress\\"'],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
    ],
    deps = [
        "//cyber",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/common/proto:header_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "//modules/drivers/tools/image_decompress/proto:config_cc_proto",
    ],
)

cc_test(
    name = "image_decompress_test",
    size = "small",
    srcs = ["image_decompress_test.cc"],
    deps = [
        ":image_decompress_lib",
        "@com_google_googletest//:gtest_main",
        "@opencv",
    ],
)

cc_binary(
    name = "libimage_decompress.so",
    linkshared = True,
    linkstatic = False,
    deps = [":image_decompress_lib"],
)

cpplint()
