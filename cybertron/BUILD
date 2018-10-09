load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "mainboard",
    srcs = glob([
        "mainboard/*.cpp",
        "mainboard/*.h",
    ]),
    deps = [
        ":cybertron_core",
    ],
    copts = [
        "-Iframework",
    ],
    linkstatic = False,
)

cc_library(
    name = "cybertron",
    deps = [
        ":cybertron_core",
    ],
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
    copts = [
        "-Iframework",
    ],
)

cc_library(
    name = "cybertron_core",
    srcs = glob([
            "*.cpp",
    ]),
    deps = [
          "//cybertron/blocker:blocker_lib",
          "//cybertron/common:common_lib",
          "//cybertron/component:component_lib",
          "//cybertron/class_loader:class_loader_lib",
          "//cybertron/croutine:croutine_lib",
          "//cybertron/event:event_lib",
          "//cybertron:init_lib",
          "//cybertron/logger:logger_lib",
          "//cybertron/node:node_lib",
          "//cybertron/proto:proto_lib",
          "//cybertron/parameter:parameter_lib",
          "//cybertron/record:record_lib",
          "//cybertron/scheduler:scheduler_lib",
          "//cybertron/task:task_lib",
          "//third_party/tf2:tf2",
          "@fastrtps",
    ],
    copts = [
        "-Iframework",
    ],
    linkopts = [
        "-luuid",
        "-lprotobuf",
        "-lglog",
        "-lgflags",
    ],
)


cpplint()
