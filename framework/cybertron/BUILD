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
          "//framework/cybertron/blocker:blocker_lib",
          "//framework/cybertron/common:common_lib",
          "//framework/cybertron/component:component_lib",
          "//framework/cybertron/class_loader:class_loader_lib",
          "//framework/cybertron/croutine:croutine_lib",
          "//framework/cybertron/event:event_lib",
          "//framework/cybertron:init_lib",
          "//framework/cybertron/logger:logger_lib",
          "//framework/cybertron/node:node_lib",
          "//framework/cybertron/proto:proto_lib",
          "//framework/cybertron/parameter:parameter_lib",
          "//framework/cybertron/record:record_lib",
          "//framework/cybertron/scheduler:scheduler_lib",
          "//framework/cybertron/task:task_lib",
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
