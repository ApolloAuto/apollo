load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
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
    alwayslink = True,
)

cc_binary(
    name = "libstorytelling_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":storytelling_lib"],
)

install(
    name = "install",
    library_dest = "storytelling/lib",
    data_dest = "storytelling",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":storytelling.BUILD",
    ],
    targets = [
        ":libstorytelling_component.so",
        "//modules/storytelling/proto:py_pb_storytelling"
    ],
    deps = [
        ":pb_hdrs",
    ]
)

install(
    name = "pb_hdrs",
    data_dest = "storytelling/include",
    data = [
        "//modules/storytelling/proto:storytelling_config_cc_proto",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_storytelling_src",
        ":install_storytelling_hdrs"
    ],
)

install_src_files(
    name = "install_storytelling_src",
    src_dir = ["."],
    dest = "storytelling/src",
    filter = "*",
)

install_src_files(
    name = "install_storytelling_hdrs",
    src_dir = ["."],
    dest = "storytelling/include",
    filter = "*.h",
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
