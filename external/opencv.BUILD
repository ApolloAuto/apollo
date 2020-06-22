licenses(["notice"])

package(default_visibility = ["//visibility:public"])

#TODO(storypku): split opencv into seperate components to speed up build
# e.g., opencv_imgproc/opencv_highgui/...

cc_library(
    name = "opencv",
    includes = ["."],
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
        "-lopencv_imgcodecs",
    ],
)
