load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libimage_decompress.so",
    srcs = [
        "image_decompress.cc",
        "image_decompress.h",
    ],
    copts = ['-DMODULE_NAME=\\"image_decompress\\"'],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
        "//modules/tools/image_decompress/proto:decompress_proto",
    ],
)

cpplint()
