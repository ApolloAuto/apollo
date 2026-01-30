load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_binary",
    srcs = ["binary.cc"],
    hdrs = ["binary.h"],
)

apollo_cc_library(
    name = "cyber_state",
    srcs = ["state.cc"],
    hdrs = ["state.h"],
    deps = ["//cyber/common:cyber_common"],
)

apollo_cc_library(
    name = "cyber",
    srcs = [
        "cyber.cc",
        "init.cc",
    ],
    hdrs = [
        "cyber.h",
        "init.h",
    ],
    includes = ["."],
    linkopts = ["-lrt"],
    visibility = ["//visibility:public"],
    deps = [
        ":cyber_binary",
        ":cyber_state",
        "//cyber/base:cyber_base",
        "//cyber/blocker:cyber_blocker",
        "//cyber/class_loader:cyber_class_loader",
        "//cyber/common:cyber_common",
        "//cyber/component:cyber_component",
        "//cyber/context:cyber_context",
        "//cyber/croutine:cyber_croutine",
        "//cyber/data:cyber_data",
        "//cyber/event:cyber_event",
        "//cyber/io:cyber_io",
        "//cyber/logger:cyber_logger",
        "//cyber/message:cyber_message",
        "//cyber/node:cyber_node",
        "//cyber/parameter:cyber_parameter",
        "//cyber/plugin_manager:cyber_plugin_manager",
        "//cyber/profiler:cyber_profiler",
        "//cyber/proto:clock_cc_proto",
        "//cyber/proto:run_mode_conf_cc_proto",
        "//cyber/record:cyber_record",
        "//cyber/scheduler:cyber_scheduler",
        "//cyber/service:cyber_service",
        "//cyber/service_discovery:cyber_service_discovery",
        "//cyber/service_discovery:cyber_service_discovery_role",
        "//cyber/sysmo:cyber_sysmo",
        "//cyber/task:cyber_task",
        "//cyber/time:cyber_time",
        "//cyber/timer:cyber_timer",
        "//cyber/transport:cyber_transport",
        "@com_github_google_glog//:glog",
        "@com_google_protobuf//:protobuf",
        "@fastrtps",
    ],
)

filegroup(
    name = "cyber_conf",
    srcs = glob([
        "conf/*.conf",
    ]),
)

apollo_package()

cpplint()
