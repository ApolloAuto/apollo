load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "qt_core",
    hdrs = glob(["**/*"]),
    copts = [
        "-I.",
    ],
    includes = [
        "QtCore",
    ],
    linkopts = [
        "-Wl,-rpath,/usr/local/qt5/lib",
        "-lQt5Core",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "qt_widgets",
    hdrs = glob(["**/*"]),
    copts = [
        "-I.",
    ],
    includes = ["QtWidgets"],
    linkopts = [
        "-L/usr/local/qt5/lib",
        "-lQt5Widgets",
    ],
    visibility = ["//visibility:public"],
    deps = [":qt_core"],
)

cc_library(
    name = "qt_gui",
    hdrs = glob(["**/*"]),
    copts = [
        "-I.",
    ],
    includes = ["QtGui"],
    linkopts = [
        "-L/usr/local/qt5/lib",
        "-lQt5Gui",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":qt_core",
        ":qt_widgets",
    ],
)

cc_library(
    name = "qt_opengl",
    hdrs = glob(["**/*"]),
    copts = [
        "-I.",
    ],
    includes = ["QtOpenGL"],
    linkopts = [
        "-L/usr/local/qt5/lib",
        "-lQt5OpenGL",
        "-lGL",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":qt_core",
        ":qt_gui",
        ":qt_widgets",
        #"@opengl",
    ],
)
