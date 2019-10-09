load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_teller",
    hdrs = ["base_teller.h"],
    deps = [
        "//modules/storytelling/proto:story_proto",
    ],
)

cc_library(
    name = "close_to_junction_teller",
    srcs = ["close_to_junction_teller.cc"],
    hdrs = ["close_to_junction_teller.h"],
    deps = [
        ":base_teller",
        "//modules/common/adapters:adapter_gflags",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/storytelling:frame_manager",
    ],
)

cpplint()
