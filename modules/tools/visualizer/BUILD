load("//tools:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_visualizer",
    includes = [
        ".",
    ],
    linkopts = [
        "-lcybertron",
        "-lcybertron_proto",
        "-lcybertron_common",
        "-lfastrtps",
        "-lconsole_bridge",
        "-lprotobuf",
        "-lglog",
        "-lpthread",
        "-lGL",
    ],
    deps = [
        ":visualizer_lib",
        "//framework:cybertron",
        "@qt//:qt_core",
        "@qt//:qt_widgets",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
    ],
)

# name, src, hdr, uis = [], res = [], normal_hdrs = [], deps = None
qt_cc_library(
    name = "visualizer_lib",

    src = glob(
        [ "*.cc"],
    ),

    hdr = glob([
        "*.h",
    ]),

    uis = glob([
        "uis/*.ui", 
    ]),

    res = glob([
        "*.qrc",
    ]),

    includes = [
        ".",
    ],

    copts=[
        "-Imodules/drivers/proto",
    ],

    deps = [
        "//modules/drivers/proto:sensor_proto",
        "//framework:cybertron",
        "@qt//:qt_core",
        "@qt//:qt_widgets",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
    ],

    linkstatic = 1
)
