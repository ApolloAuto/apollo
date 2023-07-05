load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "play_param",
    hdrs = ["play_param.h"],
)

cc_library(
    name = "play_task",
    srcs = ["play_task.cc"],
    hdrs = ["play_task.h"],
    deps = [
        "//cyber/common:log",
        "//cyber/message:raw_message",
        "//cyber/node:writer",
    ],
)

cc_library(
    name = "play_task_buffer",
    srcs = ["play_task_buffer.cc"],
    hdrs = ["play_task_buffer.h"],
    deps = [
        ":play_task",
    ],
)

cc_library(
    name = "play_task_consumer",
    srcs = ["play_task_consumer.cc"],
    hdrs = ["play_task_consumer.h"],
    deps = [
        ":play_task_buffer",
        "//cyber/common:log",
        "//cyber/time",
    ],
)

cc_library(
    name = "play_task_producer",
    srcs = ["play_task_producer.cc"],
    hdrs = ["play_task_producer.h"],
    deps = [
        ":play_param",
        ":play_task_buffer",
        "//cyber",
        "//cyber/common:log",
        "//cyber/message:protobuf_factory",
        "//cyber/message:raw_message",
        "//cyber/node",
        "//cyber/node:writer",
        "//cyber/record:record_reader",
        "//cyber/record:record_viewer",
    ],
)

cc_library(
    name = "player",
    srcs = ["player.cc"],
    hdrs = ["player.h"],
    deps = [
        ":play_param",
        ":play_task_buffer",
        ":play_task_consumer",
        ":play_task_producer",
        "//cyber:init",
    ],
)

cpplint()
