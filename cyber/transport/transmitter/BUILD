load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "transmitter",
  deps = [
    ":hybrid_transmitter",
    ":intra_transmitter",
    ":rtps_transmitter",
    ":shm_transmitter",
    ],
    )

cc_library(
    name = "hybrid_transmitter",
    hdrs = ["hybrid_transmitter.h"],
    deps = [
        ":transmitter_interface",
    ],
)

cc_library(
    name = "intra_transmitter",
    hdrs = ["intra_transmitter.h"],
    deps = [
        ":transmitter_interface",
    ],
)

cc_library(
    name = "transmitter_interface",
    hdrs = ["transmitter.h"],
    deps = [
        "//cyber/event:perf_event_cache",
        "//cyber/transport/common:endpoint",
        "//cyber/transport/message:message_info",
    ],
)

cc_library(
    name = "rtps_transmitter",
    hdrs = ["rtps_transmitter.h"],
    deps = [
        ":transmitter_interface",
    ],
)

cc_library(
    name = "shm_transmitter",
    hdrs = ["shm_transmitter.h"],
    deps = [
        ":transmitter_interface",
    ],
)

cpplint()
