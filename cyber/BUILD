load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cyber",
    deps = [
        "//cyber:cyber_core",
    ],
    linkstatic = False,
)

cc_binary(
    name = "mainboard",
    srcs = glob([
        "mainboard/*.cc",
        "mainboard/*.h",
    ]),
    deps = [
        ":cyber_core",
        "//cyber/proto:dag_config_cc_proto",
    ],
    copts = [
        "-pthread",
    ],
    linkstatic = False,
)

cc_binary(
    name = "libcyber.so",
    deps = [
        ":cyber_core",
          "@fastrtps//:fastrtps",
    ],
    #TODO: Using deps instead.
    linkopts = [
        "-luuid",
    ],
    linkshared = True,
    linkstatic = True,
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
)

cc_library(
    name = "init",
    hdrs = [
        "cyber.h",
        "init.h",
    ],
    deps = [
        "state",
    ],
)

cc_library(
    name = "cyber_core",
    srcs = [
        "cyber.cc",
        "init.cc",
    ],
    deps = [
          "//cyber:binary",
          "//cyber:state",
          "//cyber/base",
          "//cyber/blocker:blocker_manager",
          "//cyber/common",
          "//cyber/component:component",
          "//cyber/component:timer_component",
          "//cyber/class_loader:class_loader",
          "//cyber/class_loader:class_loader_manager",
          "//cyber/croutine:croutine",
          "//cyber/data:data",
          "//cyber/event:perf_event",
          "//cyber/event:perf_event_cache",
          "//cyber:init",
          "//cyber/logger:logger",
          "//cyber/logger:async_logger",
          "//cyber/message:message_traits",
          "//cyber/message:raw_message_traits",
          "//cyber/message:py_message_traits",
          "//cyber/message:protobuf_traits",
          "//cyber/message:intra_message_traits",
          "//cyber/node:node",
          "//cyber/proto:run_mode_conf_cc_proto",
          "//cyber/parameter:parameter_client",
          "//cyber/parameter:parameter_server",
          "//cyber/record:record",
          "//cyber/scheduler:scheduler",
          "//cyber/service:client",
          "//cyber/service:service",
          "//cyber/service_discovery:topology_manager",
          "//cyber/task:task",
          "//cyber/time:time",
          "//cyber/time:duration",
          "//cyber/time:rate",
          "//cyber/timer:timer",
          "//cyber/transport:transport_lib",
          "//third_party/tf2:tf2",
          "@fastrtps",
    ],
    linkopts = [
        "-luuid",
        "-lprotobuf",
        "-lglog",
        "-lgflags",
    ],
)


cpplint()
