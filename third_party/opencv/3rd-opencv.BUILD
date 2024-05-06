load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

#TODO(storypku): split opencv into seperate components to speed up build
# e.g., opencv_imgproc/opencv_highgui/...

cc_library(
    name = "core",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lopencv_core",
    ],
    include_prefix = "opencv2",
    strip_include_prefix = "include",
)

cc_library(
    name = "highgui",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lopencv_highgui",
    ],
    # Note(storypku): dependency relation derived from ldd
    deps = [
        ":core",
        ":imgproc",
    ],
    include_prefix = "opencv2",
    strip_include_prefix = "include",
)

cc_library(
    name = "imgproc",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lopencv_imgproc",
    ],
    deps = [
        ":core",
    ],
    include_prefix = "opencv2",
    strip_include_prefix = "include",
)

cc_library(
    name = "imgcodecs",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lopencv_imgcodecs",
    ],
    deps = [
        ":core",
        ":imgproc",
    ],
    include_prefix = "opencv2",
    strip_include_prefix = "include",
)

cc_library(
    name = "calib3d",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lopencv_calib3d", 
    ],
    deps = [
        ":core",
        ":imgproc",
    ],
    include_prefix = "opencv2",
    strip_include_prefix = "include",
)