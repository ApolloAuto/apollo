licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "caffe",
    includes = [
        ".",
        "/usr/include",
    ],
    linkopts = [
        "-lboost_system",
        "-lboost_thread",
        "-lboost_filesystem",
        "-lpthread",
        "-lblas",
        "-lcblas",
        "-lz",
        "-ldl",
        "-lm",
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
        "-lcaffe",
    ],
)
