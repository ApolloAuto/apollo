load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sim_control",
    srcs = [
        "sim_control.cc",
    ],
    hdrs = [
        "sim_control.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/dreamview/backend/common:dreamview_gflags",
    ],
)

cpplint()
