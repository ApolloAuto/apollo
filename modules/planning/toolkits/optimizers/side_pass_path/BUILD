load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "side_pass_path_optimizer",
    srcs = [
        "side_pass_path_optimizer.cc",
    ],
    hdrs = [
        "side_pass_path_optimizer.h",
    ],
    deps = [
        "//modules/planning/toolkits/optimizers:path_optimizer",
        "//modules/planning/toolkits/optimizers/road_graph",
    ],
)

cpplint()
