load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtps_lib",
    srcs = glob([
        "*.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron/common",
        "//cybertron/base:atomic_hash_map",
        "//cybertron/base:atomic_rw_lock",
        "//cybertron/proto:qos_profile_cc_proto",
        "//cybertron/proto:transport_conf_cc_proto",
        "//cybertron/transport/common:transport_common",
        "//cybertron/transport/message:transport_message_lib",
        "//cybertron/transport/qos:qos_lib",
        "@fastrtps//:fastrtps",
    ],
)

cpplint()
