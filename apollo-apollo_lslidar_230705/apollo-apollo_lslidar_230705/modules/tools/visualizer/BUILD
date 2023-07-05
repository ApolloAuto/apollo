load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//third_party/qt5:qt.bzl", "qt_cc_library")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "tools/bin",
    targets = [
        ":cyber_visualizer",
    ],
    deps = [
        "//cyber:install",
    ],
)

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
    linkstatic = True,
    deps = [
        ":visualizer_lib",
        "@fastrtps",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
    ],
)

# TODO(all): Extract the large library to thin pieces.
qt_cc_library(
    name = "visualizer_lib",
    srcs = glob(
        ["*.cc"],
    ),
    hdrs = glob([
        "*.h",
    ]),
    copts = [
        "-Iexternal/qt",
    ],
    includes = [
        ".",
    ],
    linkstatic = False,
    res = glob([
        "*.qrc",
    ]),
    uis = glob([
        "uis/*.ui",
    ]),
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/sensor_msgs:radar_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
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
