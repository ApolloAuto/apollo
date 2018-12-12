load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "image_decompress_component_lib",
    srcs = [
        "image_decompress.cc",
    ],
    hdrs = [
        "image_decompress.h",
    ],
    deps = [
        "//cyber",
        "//modules/drivers/camera/proto:camera_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/tools/image_decompress/proto:decompress_proto",
        "@opencv2//:core",
        "@opencv2//:highgui"
    ],
    copts = ['-DMODULE_NAME=\\"image_decompress\\"']
)

cc_binary(
    name = "libimage_decompress.so",
    deps = [":image_decompress_component_lib"],
    linkshared = True,
    linkstatic = False,
)

cpplint()
