load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary", "apollo_qt_library")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "cyber_recorder_gui_simple",
    srcs = ["main.cpp"],
    copts = [
        "-Iexternal/qt",
        "-std=c++17",
        "-fPIC",
    ],
    linkopts = [
        "-pthread",
    ],
    deps = [
        ":cyber_recorder_gui_simple_lib",
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_widgets",
    ],
)

apollo_qt_library(
    name = "cyber_recorder_gui_simple_lib",
    srcs=["mainwindow.cpp",
          "record_playbar.cpp",
          "info.cc",
          "doubleslider.cc",
          "player/play_task_buffer.cc",
          "player/play_task_consumer.cc",
          "player/play_task_producer.cc",
          "player/play_task.cc",
          "player/player.cc"],
    hdrs=["mainwindow.h",
          "record_playbar.h",
          "info.h",
          "doubleslider.h",
          "player/play_param.h",
          "player/play_task_buffer.h",
          "player/play_task_consumer.h",
          "player/play_task_producer.h",
          "player/play_task.h",
          "player/player.h"],
    #srcs = glob(["*.cpp","*.cc"]),
    #hdrs = glob(["*.h"]),
    uis = glob(["*.ui"]),
    copts = [
        "-Iexternal/qt",
        "-std=c++17",
        "-fPIC",
    ],
    includes = [
        ".",
    ],
    linkstatic = False,
    deps = [
        "//cyber",
        "//cyber/common:cyber_common",
        "//cyber/proto:record_cc_proto",
        "@qt//:qt_core",
        "@qt//:qt_gui",
        "@qt//:qt_widgets",
    ],
)


# TODO(all): Disable linter temporarily as the generated ui files should be
# excluded from check. But we should also check the .h and .cc files, if they
# are extracted to their own cc_libraries. See the TODO above.
# cpplint()
apollo_package(enable_source=False)
apollo_package()
