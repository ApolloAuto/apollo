load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

#TODO(storypku): split opencv into seperate components to speed up build
# e.g., opencv_imgproc/opencv_highgui/...

cc_library(
    name = "core",
    includes = ["include"],
    linkopts = [
        "-lopencv_core",
    ],
    strip_include_prefix = "include/opencv4",
)

cc_library(
    name = "highgui",
    includes = ["include"],
    linkopts = [
        "-lopencv_highgui",
    ],
    # Note(storypku): dependency relation derived from ldd
    deps = [
        ":core",
        ":imgproc",
    ],
    strip_include_prefix = "include/opencv4",
)

cc_library(
    name = "imgproc",
    includes = ["include"],
    linkopts = [
        "-lopencv_imgproc",
    ],
    deps = [
        ":core",
    ],
    strip_include_prefix = "include/opencv4",
)

cc_library(
    name = "imgcodecs",
    includes = ["include"],
    linkopts = [
        "-lopencv_imgcodecs",
    ],
    deps = [
        ":core",
        ":imgproc",
    ],
    strip_include_prefix = "include/opencv4",
)
