load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "frame_manager",
    srcs = ["frame_manager.cc"],
    hdrs = ["frame_manager.h"],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
    ],
)

cc_library(
    name = "storytelling_lib",
    srcs = ["storytelling.cc"],
    hdrs = ["storytelling.h"],
    copts = ['-DMODULE_NAME=\\"storytelling\\"'],
    deps = [
        "//cyber",
        "//modules/storytelling/story_tellers:close_to_junction_teller",
    ],
)

cc_binary(
    name = "libstorytelling.so",
    linkshared = True,
    linkstatic = False,
    deps = [":storytelling_lib"],
)

cpplint()
