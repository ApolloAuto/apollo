licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lib",
    includes = [
        ".",
        "/usr/include",
    ],
    linkopts = [
        "-Wl,-rpath,/usr/lib/x86_64-linux-gnu/",
        "-lboost_system",
        "-lboost_thread",
        "-lboost_filesystem",
        "-lpthread",
        "-lblas",
        "-lcblas",
        "-lhdf5_hl",
        "-lhdf5",
        "-lz",
        "-ldl",
        "-lm",
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
        "-lcaffe",
    ],
)
