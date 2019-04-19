load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "video2jpg",
    srcs = [
        "video2jpg.cc",
    ],
    deps = [
        ":frame_processor_lib",
        "//cyber/common:log",
        "//external:gflags",
    ],
)

cc_library(
    name = "frame_processor_lib",
    hdrs = [
        "frame_processor.h",
    ],
    srcs = [
        "frame_processor.cc",
    ],
    linkstatic = True,
    deps = [
        ":h265_decoder_lib",
        "//cyber/common:log",
        "@ffmpeg",
    ],
)

cc_library(
    name = "h265_decoder_lib",
    hdrs = [
        "h265_decoder.h",
    ],
    srcs = [
        "h265_decoder.cc",
    ],
    linkstatic = True,
    deps = [
        "//cyber/common:log",
        "@ffmpeg",
    ],
)

cpplint()
