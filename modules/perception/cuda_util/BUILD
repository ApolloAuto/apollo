load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "cuda_util",
    srcs = [
        "build/libcuda_util.so",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "@cuda",
    ],
)

cpplint()
