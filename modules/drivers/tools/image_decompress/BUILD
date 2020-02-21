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
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/tools/image_decompress/proto:decompress_proto",
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
    linkstatic = False,
    deps = [":image_decompress_lib"],
)

cpplint()
