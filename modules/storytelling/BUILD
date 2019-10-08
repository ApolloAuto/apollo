load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "frame_manager",
    hdrs = ["frame_manager.h"],
    srcs = ["frame_manager.cc"],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/storytelling/proto:story_proto",
    ],
)

cpplint()
