load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//third_party/qt5:qt.bzl", "qt_cc_library")

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
        "-pthread",
    ],
    deps = [
        ":visualizer_lib",
        "//third_party:console_bridge",
        "//third_party:glvnd",
        "@fastrtps",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
    ],
)

# TODO(all): Extract the large library to thin pieces.
# name, src, hdr, uis = [], res = [], normal_hdrs = [], deps = None
qt_cc_library(
    name = "visualizer_lib",
    src = glob(
        ["*.cc"],
    ),
    copts = [
        "-Imodules/drivers/proto",
        "-Iexternal/qt",
        "-fPIC",
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

# TODO(all): Disable linter temporarily as the generated ui files should be
# excluded from check. But we should also check the .h and .cc files, if they
# are extracted to their own cc_libraries. See the TODO above.
# cpplint()
