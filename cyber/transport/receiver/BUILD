load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "receiver",
    deps = [
        ":hybrid_receiver",
        ":intra_receiver",
        ":rtps_receiver",
        ":shm_receiver",
    ],
)

cc_library(
    name = "receiver_interface",
    hdrs = ["receiver.h"],
    deps = [
        "//cyber/transport/common:endpoint",
        "//cyber/transport/message:history",
        "//cyber/transport/message:message_info",
    ],
)

cc_library(
    name = "hybrid_receiver",
    hdrs = ["hybrid_receiver.h"],
    deps = [
        ":receiver_interface",
    ],
)

cc_library(
    name = "intra_receiver",
    hdrs = ["intra_receiver.h"],
    deps = [
        ":receiver_interface",
    ],
)

cc_library(
    name = "rtps_receiver",
    hdrs = ["rtps_receiver.h"],
    deps = [
        ":receiver_interface",
    ],
)

cc_library(
    name = "shm_receiver",
    hdrs = ["shm_receiver.h"],
    deps = [
        ":receiver_interface",
        "//cyber/transport/shm:readable_info",
    ],
)

cpplint()
