load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_visualizer",
    copts = [
        "-Iexternal/qt",
    ],
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
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
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
        "-Iexternal/qt",
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
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/drivers/proto:radar_cc_proto",
        "//modules/drivers/proto:sensor_image_cc_proto",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
    ],
)

cpplint()
