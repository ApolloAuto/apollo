load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cybertron",
    deps = [
        "//cybertron:cybertron_core",
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
        ":cybertron_core",
        "//cybertron/proto:dag_config_cc_proto",
    ],
    copts = [
        "-pthread",
    ],
    linkstatic = False,
)

cc_binary(
    name = "libcybertron.so",
    deps = [
        ":cybertron_core",
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
        "cybertron.h",
        "init.h",
    ],
    deps = [
        "state",
    ],
)

cc_library(
    name = "cybertron_core",
    srcs = [
        "cybertron.cc",
        "init.cc",
    ],
    deps = [
          "//cybertron:binary",
          "//cybertron:state",
          "//cybertron/base",
          "//cybertron/blocker:blocker_manager",
          "//cybertron/common",
          "//cybertron/component:component",
          "//cybertron/component:timer_component",
          "//cybertron/class_loader:class_loader",
          "//cybertron/class_loader:class_loader_manager",
          "//cybertron/croutine:croutine",
          "//cybertron/data:data",
          "//cybertron/event:perf_event",
          "//cybertron/event:perf_event_cache",
          "//cybertron:init",
          "//cybertron/logger:logger",
          "//cybertron/logger:async_logger",
          "//cybertron/message:message_traits",
          "//cybertron/message:raw_message_traits",
          "//cybertron/message:py_message_traits",
          "//cybertron/message:protobuf_traits",
          "//cybertron/message:intra_message_traits",
          "//cybertron/node:node_lib",
          "//cybertron/proto:run_mode_conf_cc_proto",
          "//cybertron/parameter:parameter_client",
          "//cybertron/parameter:parameter_server",
          "//cybertron/record:record_lib",
          "//cybertron/scheduler:scheduler",
          "//cybertron/service:client",
          "//cybertron/service:service",
          "//cybertron/service_discovery:topology_manager",
          "//cybertron/task:task",
          "//cybertron/time:time",
          "//cybertron/time:rate",
          "//cybertron/timer:timer_lib",
          "//cybertron/transport:transport_lib",
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
