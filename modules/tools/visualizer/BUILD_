load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary", "apollo_qt_library")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "cyber_visualizer",
    copts = [
        "-Iexternal/qt",
    ],
    linkopts = [
        "-pthread",
    ],
    deps = [
        ":visualizer_lib",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_opengl",
        "@qt//:qt_widgets",
    ],
)

filegroup(
    name = "resources",
    srcs = glob(["images/*", "shaders/*"]),
)

# TODO(all): Extract the large library to thin pieces.
apollo_qt_library(
    name = "visualizer_lib",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    data = [":resources"],
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
        "@fastrtps",
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

apollo_package()