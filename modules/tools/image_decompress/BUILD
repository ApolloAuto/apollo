load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "image_decompress_lib",
    srcs = [
        "image_decompress.cc",
    ],
    hdrs = [
        "image_decompress.h",
    ],
    copts = ['-DMODULE_NAME=\\"image_decompress\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/tools/image_decompress/proto:decompress_proto",
        "@opencv2//:core",
        "@opencv2//:highgui",
    ],
)

cc_test(
    name = "image_decompress_test",
    size = "small",
    srcs = ["image_decompress_test.cc"],
    deps = [
        ":image_decompress_lib",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "libimage_decompress.so",
    linkshared = True,
    linkstatic = False,
    deps = [":image_decompress_lib"],
)

cpplint()
