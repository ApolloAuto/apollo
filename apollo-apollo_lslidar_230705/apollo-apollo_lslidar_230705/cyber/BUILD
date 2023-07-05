load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
        #"//cyber/python/cyber_py3:runtime_files",
        "//cyber/proto:runtime_files",
        ":cyberfile.xml",
        "cyber.BUILD"
    ],
    data_dest = "cyber",
    targets = [
        ":libcyber_core.so",
    ],
    library_dest = "cyber/lib",
    rename = {
        "cyber/.release.bash": "setup.bash",
        #"cyber/python/cyber_py3/cyber_py3.BUILD": "BUILD",
        "cyber/proto/py_pb2.BUILD": "BUILD",
    },
    deps = [
        "//cyber/python/cyber_py3:cyber_python_library",
        "//cyber/proto:pb_cyber",
        "//cyber/proto:cyber_proto_hdrs",
        "//cyber/proto:cyber_proto_so",
        "//cyber/mainboard:install",
        "//cyber/python/internal:install",
        "//cyber/tools:install",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_cyber_src",
        ":install_cyber_hdrs"
    ],
)

install_src_files(
    name = "install_cyber_src",
    src_dir = ["."],
    dest = "cyber/src",
    filter = "*",
)

install_src_files(
    name = "install_cyber_hdrs",
    src_dir = ["."],
    dest = "cyber/include",
    filter = "*.h",
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
    hdrs = ["cyber.h"],
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
