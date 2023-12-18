load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

#TODO(storypku): split opencv into seperate components to speed up build
# e.g., opencv_imgproc/opencv_highgui/...

cc_library(
    name = "core",
    includes = ["."],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lopencv_core",
    ],
)

cc_library(
    name = "highgui",
    includes = ["."],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lopencv_highgui",
    ],
    # Note(storypku): dependency relation derived from ldd
    deps = [
        ":core",
        ":imgproc",
    ],
)

cc_library(
    name = "imgproc",
    includes = ["."],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lopencv_imgproc",
    ],
    deps = [
        ":core",
    ],
)

cc_library(
    name = "imgcodecs",
    includes = ["."],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lopencv_imgcodecs",
    ],
    deps = [
        ":core",
        ":imgproc",
    ],
)

cc_library(
    name = "calib3d",
    includes = ["."],
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lopencv_calib3d",
    ],
    deps = [
        ":core",
    ],
)
