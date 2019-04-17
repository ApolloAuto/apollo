load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "video2jpg",
    deps = [
        ":video2jpg_lib",
    ], 
)

cc_library(
    name = "video2jpg_lib",
    includes = [
        ".",
    ],
    hdrs = [
        "h265_decoder.h",
    ],
    srcs = [
        "video2jpg.cc",
        "h265_decoder.cc",
    ],
    linkstatic = True,
    deps = [
        "@ffmpeg",
        "//external:gflags",
    ],
)

cpplint()
