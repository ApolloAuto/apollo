load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_storytelling",
    srcs = [
        "frame_manager.cc",
        "story_tellers/close_to_junction_teller.cc",
        "common/storytelling_gflags.cc"
    ],
    hdrs = [
        "frame_manager.h",
        "story_tellers/base_teller.h",
        "story_tellers/close_to_junction_teller.h",
        "common/storytelling_gflags.h"
    ],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/common_msgs/storytelling_msgs:story_cc_proto",
        "//modules/storytelling/proto:storytelling_config_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/map:apollo_map",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@com_github_gflags_gflags//:gflags"
    ]
)

apollo_component(
    name = "libstorytelling_component.so",
    srcs = ["storytelling.cc"],
    hdrs = ["storytelling.h"],
    copts = ['-DMODULE_NAME=\\"storytelling\\"'],
    deps = [
        "//cyber",
        ":apollo_storytelling",
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

apollo_package()
cpplint()
