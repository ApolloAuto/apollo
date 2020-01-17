load("//tools:cpplint.bzl", "cpplint")
load("//tools:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_visualizer",
    includes = [
        ".",
    ],
    linkopts = [
        "-lconsole_bridge",
        "-lGL",
        "-lQt5Core",
        "-lQt5Widgets",
        "-lQt5Gui",
        "-lQt5OpenGL",
    ],
    deps = [
        ":visualizer_lib",
        "//third_party:pthread",
        "@fastrtps",
    ],
)

# name, src, hdr, uis = [], res = [], normal_hdrs = [], deps = None
qt_cc_library(
    name = "visualizer_lib",
    src = glob(
        ["*.cc"],
    ),
    copts = [
        "-Imodules/drivers/proto",
        "-I/usr/include/x86_64-linux-gnu/qt5",
    ],
    hdr = glob([
        "*.h",
    ]),
    includes = [
        ".",
    ],
    linkstatic = 1,
    res = glob([
        "*.qrc",
    ]),
    uis = glob([
        "uis/*.ui",
    ]),
    deps = [
        "//cyber",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cpplint()
