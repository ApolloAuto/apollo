load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
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
    name = "libstorytelling_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":storytelling_lib"],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libstorytelling_component.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cpplint()
