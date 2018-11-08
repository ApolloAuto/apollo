load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_recorder",
    srcs = [ "main.cc", ],
    deps = [
        "info",
        "player",
        "recorder",
        "recoverer",
        "spliter",
        "//cyber:init",
        "//cyber/common:file",
        "//cyber/common:time_conversion",
    ],
    linkopts = [
        "-pthread",
    ],
)

cc_library(
    name = "info",
    srcs = [ "info.cc", ],
    hdrs = [ "info.h", ],
    deps = [
        "//cyber/common:time_conversion",
        "//cyber/proto:record_cc_proto",
        "//cyber/record:record_file_reader",
    ],
)

cc_library(
    name = "player",
    srcs = [
        "player/play_task.cc",
        "player/play_task_consumer.cc",
        "player/player.cc",
        "player/play_task_buffer.cc",
        "player/play_task_producer.cc",
        "player/terminal_controller.cc",
    ],
    hdrs = [
        "player/play_param.h",
        "player/play_task.h",
        "player/play_task_consumer.h",
        "player/player.h",
        "player/play_task_buffer.h",
        "player/play_task_producer.h",
        "player/terminal_controller.h",
    ],
    deps = [
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
        "//cyber:cyber",
        "//cyber/record:record_reader",
    ]
)

cc_library(
    name = "recorder",
    srcs = [ "recorder.cc", ],
    hdrs = [ "recorder.h", ],
    deps = [
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
        "//cyber:cyber",
        "@fastrtps//:fastrtps",
    ],
)

cc_library(
    name = "recoverer",
    srcs = [ "recoverer.cc", ],
    hdrs = [ "recoverer.h", ],
    deps = [
        "//cyber/base:for_each",
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
        "//cyber/record:header_builder",
        "//cyber/record:record_file_reader",
        "//cyber/record:record_writer",
    ],
)

cc_library(
    name = "spliter",
    srcs = [ "spliter.cc", ],
    hdrs = [ "spliter.h", ],
    deps = [
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
        "//cyber/record:header_builder",
        "//cyber/record:record_file_reader",
        "//cyber/record:record_writer",
    ],
)

cpplint()
