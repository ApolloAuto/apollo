load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_binary(
    name = "cyber_recorder",
    srcs = [
        "main.cc",
    ],
    linkopts = [
        "-pthread",
        "-lprofiler",
        "-ltcmalloc",
    ],
    deps = [
        ":recorder",
        "//cyber",
        "//cyber/proto:record_cc_proto",
    ],
)

apollo_cc_library(
    name = "recorder",
    srcs = [
        "recorder.cc", "info.cc",  "recoverer.cc", 
        "spliter.cc", "player/play_task.cc", "player/play_task_buffer.cc", 
        "player/play_task_consumer.cc", "player/play_task_producer.cc", 
        "player/player.cc",
    ],
    hdrs = [
        "recorder.h", "info.h", "recoverer.h", "spliter.h", 
        "player/play_param.h", "player/play_task.h", 
        "player/play_task_buffer.h", "player/play_task_consumer.h", 
        "player/play_task_producer.h", "player/player.h",
    ],
    deps = [
        "//cyber",
        "//cyber/common:cyber_common",
        "//cyber/proto:record_cc_proto",
        "@fastrtps",
    ],
)

apollo_package()

cpplint()