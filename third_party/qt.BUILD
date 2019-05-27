cc_library(
    name = "qt_core",
    hdrs = glob(["*"]),
    copts = [
        "-Iinclude",
        "-Iinclude/QtCore",
    ],
    includes = [
        "include",
        "include/QtCore",
    ],
    linkopts = [
        "-Wl,-rpath,/usr/local/Qt5.5.1/5.5/gcc_64/lib",
        "-lQt5Core",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "qt_widgets",
    hdrs = glob(["*"]),
    copts = [
        "-Iinclude",
        #        "-Iinclude/QtCore",
        "-Iinclude/QtWidgets",
    ],
    includes = ["include/QtWidgets"],
    linkopts = [
        "-L/usr/local/Qt5.5.1/5.5/gcc_64/lib",
        "-lQt5Widgets",
    ],
    visibility = ["//visibility:public"],
    deps = [":qt_core"],
)

cc_library(
    name = "qt_gui",
    hdrs = glob(["*"]),
    copts = [
        "-Iinclude",
        # "-Iinclude/QtCore",
        "-Iinclude/QtGui",
        # "-Iinclude/QtWidgets",
    ],
    includes = ["include/QtGui"],
    linkopts = [
        "-L/usr/local/Qt5.5.1/5.5/gcc_64/lib",
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
    hdrs = glob(["*"]),
    copts = [
        "-Iinclude",
        "-Iinclude/QtCore",
        "-Iinclude/QtWidgets",
        "-Iinclude/QtGui",
        "-Iinclude/QtOpenGL",
    ],
    includes = ["include/QtOpenGL"],
    linkopts = [
        "-L/usr/local/Qt5.5.1/5.5/gcc_64/lib",
        "-lQt5OpenGL",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":qt_core",
        ":qt_gui",
        ":qt_widgets",
    ],
)
