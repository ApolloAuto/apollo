load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtps_lib",
    srcs = glob([
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cyber/common",
        "//cyber/base:atomic_hash_map",
        "//cyber/base:atomic_rw_lock",
        "//cyber/proto:qos_profile_cc_proto",
        "//cyber/proto:transport_conf_cc_proto",
        "//cyber/transport/common:transport_common",
        "//cyber/transport/message:transport_message_lib",
        "//cyber/transport/qos:qos_lib",
        "@fastrtps//:fastrtps",
    ],
)

cpplint()
