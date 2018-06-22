load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = [
        "camera.cc",
        "frame.cc",
        "gl_raster_text.cc",
    ],
    hdrs = [
        "bmp.h",
        "camera.h",
        "frame.h",
        "gl_raster_text.h",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "@eigen",
        "@glew",
        "@glfw",
    ],
)

cpplint()
