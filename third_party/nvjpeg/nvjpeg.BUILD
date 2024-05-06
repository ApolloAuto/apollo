load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "nvjpeg",
    srcs = select({
        "@platforms//cpu:aarch64": glob(
            ["jetson_multimedia_api/samples/common/classes/*.cpp"], 
            exclude = [
                "jetson_multimedia_api/samples/common/classes/NvDrmRenderer.cpp",
                "jetson_multimedia_api/samples/common/classes/NvJpegDecoder.cpp",
            ],
        ),
        "//conditions:default": [],
    }),
    alwayslink = True,
    linkopts = select({
        "@platforms//cpu:aarch64": [
            "-L/usr/lib/aarch64-linux-gnu/tegra",
            "-lnvjpeg",
            "-lnvbufsurface",
            "-lnvbufsurftransform",
            "-lv4l2",
            "-lEGL",
            "-lGLESv2",
            "-lX11",
        ],
        "//conditions:default": [],
    }), 
    deps = select({
        "@platforms//cpu:aarch64": [":common_headers"],
        "//conditions:default": [],
    }),  
)

cc_library(
    name = "common_headers",
    hdrs = select({
        "@platforms//cpu:aarch64": glob(
            ["jetson_multimedia_api/include/*.h"],
            exclude = [
                "jetson_multimedia_api/include/NvDrmRenderer.h",
                "jetson_multimedia_api/include/NvJpegDecoder.h",
            ],
        ),
        "//conditions:default": [],
    }), 
    strip_include_prefix = "jetson_multimedia_api/include", 
    deps = select({
        "@platforms//cpu:aarch64": [":libjpeg8b"],
        "//conditions:default": [],
    }),  
)

cc_library(
    name = "libjpeg8b",
    hdrs = select({
        "@platforms//cpu:aarch64": glob(["jetson_multimedia_api/include/libjpeg-8b/*.h"]),
        "//conditions:default": [],
    }),
    strip_include_prefix = "jetson_multimedia_api/include/libjpeg-8b",
    copts = ["-DTEGRA_ACCELERATE"],
)
