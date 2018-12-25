load("//tools:cpplint.bzl", "cpplint")
load("//tools:qt.bzl", "qt_cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_visualizer",
    includes = [
        ".",
    ],
    linkopts = [
        "-lfastrtps",
        "-lconsole_bridge",
        "-lprotobuf",
        "-lglog",
        "-lpthread",
        "-lGL",
    ],
    deps = [
        ":visualizer_lib",
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
        "@qt//:qt_gui",
    ],
)

cpplint()
