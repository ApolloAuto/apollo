load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cyber",
    linkstatic = False,
    deps = [
      "//cyber:cyber_core",
    ],
)

cc_binary(
    name = "mainboard",
    srcs = glob([
      "mainboard/*.cc",
      "mainboard/*.h",
    ]),
    copts = [
      "-pthread",
    ],
    linkstatic = False,
    deps = [
      ":cyber_core",
      "//cyber/proto:dag_conf_cc_proto",
    ],
)

cc_library(
    name = "binary",
    hdrs = [
      "binary.h",
    ],
)

cc_library(
    name = "state",
    srcs = [
      "state.cc",
    ],
    hdrs = [
      "state.h",
    ],
    deps = [
        "//cyber/common",
    ],
)

cc_library(
    name = "init",
    srcs = [
        "init.cc",
    ],
    hdrs = [
      "init.h",
    ],
    deps = [
      "//cyber:state",
      "//cyber/node",
      "//cyber/logger:async_logger",
    ],
)

cc_library(
    name = "cyber_core",
    linkopts = [
        "-lglog",
        "-lgflags",
        "-lprotobuf",
        "-luuid",
    ],
    srcs = [
        "cyber.cc",
    ],
    hdrs = [
      "cyber.h",
    ],
    deps = [
        "//cyber:binary",
        "//cyber/base",
        "//cyber/blocker:blocker_manager",
        "//cyber/common",
        "//cyber/component",
        "//cyber/component:timer_component",
        "//cyber/class_loader",
        "//cyber/class_loader:class_loader_manager",
        "//cyber/croutine",
        "//cyber/data",
        "//cyber/event:perf_event_cache",
        "//cyber/io",
        "//cyber:init",
        "//cyber/logger",
        "//cyber/logger:async_logger",
        "//cyber/message:message_traits",
        "//cyber/message:raw_message_traits",
        "//cyber/message:py_message_traits",
        "//cyber/message:protobuf_traits",
        "//cyber/node",
        "//cyber/proto:run_mode_conf_cc_proto",
        "//cyber/parameter:parameter_client",
        "//cyber/parameter:parameter_server",
        "//cyber/record",
        "//cyber/scheduler",
        "//cyber/service:client",
        "//cyber/service",
        "//cyber/service_discovery:topology_manager",
        "//cyber:state",
        "//cyber/task",
        "//cyber/time",
        "//cyber/time:duration",
        "//cyber/time:rate",
        "//cyber/timer",
        "//cyber/transport",
        "//cyber/transport:participant",
        "//cyber/transport:sub_listener",
        "//third_party/tf2",
        "@fastrtps",
    ],
)

cpplint()
