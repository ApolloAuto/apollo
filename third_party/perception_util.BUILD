package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "perception_util",
    srcs = [
        "lib/libperception_util.so",
    ],
    hdrs = [
        "include/undistortion.h",
        "include/region_output.h",
        "include/util.h",
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
