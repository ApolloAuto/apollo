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
        "mainboard/*.cpp",
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
    name = "init_lib",
    hdrs = glob([
            "*.h",
    ]),
)

cc_library(
    name = "cybertron_core",
    srcs = glob([
            "*.cpp",
    ]),
    deps = [
          "//cybertron/blocker:blocker_lib",
          "//cybertron/base",
          "//cybertron/common",
          "//cybertron/component:component",
          "//cybertron/component:timer_component",
          "//cybertron/class_loader:class_loader_lib",
          "//cybertron/croutine:croutine",
          "//cybertron/event:event_lib",
          "//cybertron:init_lib",
          "//cybertron/logger:async_logger",
          "//cybertron/node:node_lib",
          "//cybertron/proto:run_mode_conf_cc_proto",
          "//cybertron/parameter:parameter_lib",
          "//cybertron/record:record_lib",
          "//cybertron/scheduler:scheduler_lib",
          "//cybertron/task:task",
          "//cybertron/time:time",
          "//cybertron/time:rate",
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
