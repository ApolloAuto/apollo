load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "piecewise_jerk_speed_optimizer",
    srcs = [
        "piecewise_jerk_speed_optimizer.cc",
    ],
    hdrs = [
        "piecewise_jerk_speed_optimizer.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_speed_problem",
        "//modules/planning/tasks/optimizers:speed_optimizer",
    ],
)

cpplint()
