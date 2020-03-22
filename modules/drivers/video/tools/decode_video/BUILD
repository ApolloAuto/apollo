load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "video2jpg",
    srcs = ["video2jpg.cc"],
    deps = [
        ":frame_processor",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "frame_processor",
    srcs = ["frame_processor.cc"],
    hdrs = ["frame_processor.h"],
    deps = [
        ":h265_decoder",
        "//modules/common/util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "h265_decoder",
    srcs = ["h265_decoder.cc"],
    hdrs = ["h265_decoder.h"],
    deps = [
        "//cyber/common:log",
        "@ffmpeg",
    ],
)

cpplint()
