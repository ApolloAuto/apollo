cc_library(
    name = "qt_core",
    hdrs = glob(["*"]),
    includes = ["include", "include/QtCore"],
    copts=[
        "-Iinclude",
        "-Iinclude/QtCore",
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
    includes = ["include/QtWidgets" ],
    deps = [":qt_core"],
    copts=[
        "-Iinclude",
#        "-Iinclude/QtCore",
        "-Iinclude/QtWidgets",
    ],
    linkopts = [
        "-L/usr/local/Qt5.5.1/5.5/gcc_64/lib",
        "-lQt5Widgets",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "qt_gui",
    hdrs = glob(["*"]),
    includes = ["include/QtGui"],
    deps = [":qt_core", ":qt_widgets"],
    copts=[
        "-Iinclude",
       # "-Iinclude/QtCore",
        "-Iinclude/QtGui",
       # "-Iinclude/QtWidgets",
    ],
    linkopts = [
        "-L/usr/local/Qt5.5.1/5.5/gcc_64/lib",
        "-lQt5Gui",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "qt_opengl",
    hdrs = glob(["*"]),
    includes = ["include/QtOpenGL"],
    deps = [":qt_core", ":qt_widgets", ":qt_gui"],
    copts=[
        "-Iinclude",
        "-Iinclude/QtCore",
        "-Iinclude/QtWidgets",
        "-Iinclude/QtGui",
        "-Iinclude/QtOpenGL",
    ],
    linkopts = [
        "-L/usr/local/Qt5.5.1/5.5/gcc_64/lib",
        "-lQt5OpenGL",
    ],
    visibility = ["//visibility:public"],
)
