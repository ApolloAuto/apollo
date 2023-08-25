load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "video2jpg",
    srcs = [
        "video2jpg.cc",
        "frame_processor.cc",
        "frame_processor.h",
        "h265_decoder.cc",
        "h265_decoder.h",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
        "//modules/common/util:common_util",
        "@com_google_absl//:absl",
        "@ffmpeg//:avutil",
        "@ffmpeg//:avcodec",
    ],
)

apollo_package()
cpplint()
