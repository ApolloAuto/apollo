load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools/install:install.bzl", "install", "install_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


install_files(
    name = "third_party_export_hdrs",
    dest = "lib/include",
    files = [
        "@com_google_protobuf//:protobuf_hdrs",
        "@civetweb//:civetweb_hdrs",
    ],
)

cc_library(
    name = "cyber",
    linkstatic = False,
    deps = [
        ":cyber_core",
    ],
)

install(
    name = "install",
    data = [
        ":.release.bash",
        ":cyber_conf",
        "//cyber/python/cyber_py3:runtime_files",
    ],
    rename = {
        "cyber/.release.bash": "setup.bash",
    },
    deps = [
        # todo:// move to third_party
        ":third_party_export_hdrs",
        "@ad_rss_lib//:ad_rss_export_hdrs",

        ":pb_cyber",
        ":cyber_export_hdrs",
        "//cyber/proto:cyber_proto_hdrs",
        "//cyber/mainboard:install",
        "//cyber/python/internal:install",
        "//cyber/tools:install",
    ],
)

install_files(
    name = "cyber_export_hdrs",
    dest = "cyber/include",
    files = [
        ":cyber_hdrs",
        "//cyber/component:cyber_component_hdrs",
        "//cyber/base:cyber_base_hdrs",
        "//cyber/blocker:cyber_blocker_hdrs",
        "//cyber/class_loader:cyber_class_loader_hdrs",
        "//cyber/class_loader/shared_library:cyber_class_loader_shared_library_hdrs",
        "//cyber/class_loader/utility:cyber_class_loader_utility_hdrs",
        "//cyber/common:cyber_common_hdrs",
        "//cyber/croutine:cyber_croutine_hdrs",
        "//cyber/data:cyber_data_hdrs",
        "//cyber/event:cyber_event_hdrs",
        "//cyber/io:cyber_io_hdrs",
        "//cyber/logger:cyber_logger_hdrs",
        "//cyber/message:cyber_message_hdrs",
        "//cyber/node:cyber_node_hdrs",
        "//cyber/parameter:cyber_parameter_hdrs",
        "//cyber/scheduler:cyber_scheduler_hdrs",
        "//cyber/service:cyber_service_hdrs",
        "//cyber/service_discovery:cyber_service_discovery_hdrs",
        "//cyber/service_discovery/communication:cyber_service_discovery_communication_hdrs",
        "//cyber/sysmo:cyber_sysmo_hdrs",
        "//cyber/task:cyber_task_hdrs",
        "//cyber/time:cyber_time_hdrs",
        "//cyber/timer:cyber_timer_hdrs",
        "//cyber/transport:cyber_transport_hdrs",
        "//cyber/transport/common:cyber_transport_common_hdrs",
        "//cyber/transport/dispatcher:cyber_transport_dispatcher_hdrs",
        "//cyber/transport/message:cyber_transport_message_hdrs",
        "//cyber/transport/qos:cyber_transport_qos_hdrs",
        "//cyber/transport/receiver:cyber_transport_receiver_hdrs",
        "//cyber/transport/rtps:cyber_transport_rtps_hdrs",
        "//cyber/transport/shm:cyber_transport_shm_hdrs",
        "//cyber/transport/transmitter:cyber_transport_transmitter_hdrs",
    ],
)

filegroup(
    name = "cyber_hdrs",
    srcs = glob([
        "*.h",
    ]),
)

install_files(
    name = "pb_cyber",
    dest = "cyber",
    files = [
        "//cyber/proto:record_py_pb2",
    ],
)

cc_library(
    name = "binary",
    srcs = ["binary.cc"],
    hdrs = ["binary.h"],
)

cc_library(
    name = "state",
    srcs = ["state.cc"],
    hdrs = ["state.h"],
    deps = [
        "//cyber/common",
    ],
)

cc_library(
    name = "init",
    srcs = ["init.cc"],
    hdrs = ["init.h"],
    alwayslink = True,
    deps = [
        "//cyber:binary",
        "//cyber:state",
        "//cyber/common:file",
        "//cyber/logger:async_logger",
        "//cyber/node",
        "//cyber/proto:clock_cc_proto",
        "//cyber/sysmo",
        "//cyber/time:clock",
        "//cyber/timer:timing_wheel",
    ],
)

cc_binary(
    name = "libcyber_core.so",
    srcs = glob([
        "cyber.cc",
        "cyber.h",
        ]),
    linkshared = True,
    linkstatic = True,
    linkopts = ["-lrt"],
    deps = [
        "//cyber:binary",
        "//cyber:init",
        "//cyber:state",
        "//cyber/base",
        "//cyber/blocker:blocker_manager",
        "//cyber/class_loader",
        "//cyber/class_loader:class_loader_manager",
        "//cyber/common",
        "//cyber/component",
        "//cyber/component:timer_component",
        "//cyber/croutine",
        "//cyber/data",
        "//cyber/event:perf_event_cache",
        "//cyber/io",
        "//cyber/logger",
        "//cyber/logger:async_logger",
        "//cyber/message:message_traits",
        "//cyber/message:protobuf_traits",
        "//cyber/message:py_message_traits",
        "//cyber/message:raw_message_traits",
        "//cyber/node",
        "//cyber/parameter:parameter_client",
        "//cyber/parameter:parameter_server",
        "//cyber/proto:run_mode_conf_cc_proto",
        "//cyber/record",
        "//cyber/scheduler",
        "//cyber/scheduler:scheduler_factory",
        "//cyber/service",
        "//cyber/service:client",
        "//cyber/service_discovery:topology_manager",
        "//cyber/sysmo",
        "//cyber/task",
        "//cyber/time",
        "//cyber/time:clock",
        "//cyber/time:duration",
        "//cyber/time:rate",
        "//cyber/timer",
        "//cyber/transport",
        "//cyber/transport/rtps:participant",
        "//cyber/transport/rtps:sub_listener",
        "@com_github_google_glog//:glog",
        "@com_google_protobuf//:protobuf",
        "@fastrtps",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "cyber_core",
    srcs = ["libcyber_core.so"],
    # hdrs = ["cyber.h"],
    linkopts = ["-lrt"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_github_google_glog//:glog",
        "@com_google_protobuf//:protobuf",
        "@fastrtps",
        "@com_github_gflags_gflags//:gflags",
        "//cyber/proto:run_mode_conf_cc_proto",
        "//cyber/proto:cyber_conf_cc_proto",
        "//cyber/proto:scheduler_conf_cc_proto",
        "//cyber/proto:classic_conf_cc_proto",
        "//cyber/proto:choreography_conf_cc_proto",
        "//cyber/proto:transport_conf_cc_proto",
        "//cyber/proto:perf_conf_cc_proto",
        "//cyber/proto:topology_change_cc_proto",
        "//cyber/proto:role_attributes_cc_proto",
        "//cyber/proto:proto_desc_cc_proto",
        "//cyber/proto:component_conf_cc_proto",
        "//cyber/proto:record_cc_proto",
        "//cyber/proto:parameter_cc_proto",
        "//cyber/proto:clock_cc_proto",
        "//cyber/proto:qos_profile_cc_proto",
    ],
    includes = ["."],
)
filegroup(
    name = "cyber_conf",
    srcs = glob([
        "conf/*.conf",
    ]),
)

cpplint()
