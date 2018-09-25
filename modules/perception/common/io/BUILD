load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "io_util",
    srcs = [
        "io_util.cc",
    ],    
    hdrs = [
        "io_util.h",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/perception/base:camera",
        "//modules/perception/base:distortion_model",
        "//modules/perception/base:omnidirectional_model",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/common/geometry:basic",
    ],
)

# cc_library(
#     name = "io_util_test",
#     srcs = [
#         "io_util_test.cc",
#     ],
#     deps = [
#         ":io_util",
#         "@gtest//:main",
#     ],
# )


cpplint()
