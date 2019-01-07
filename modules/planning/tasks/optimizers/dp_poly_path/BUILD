load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dp_poly_path_optimizer",
    srcs = [
        "dp_poly_path_optimizer.cc",
    ],
    hdrs = [
        "dp_poly_path_optimizer.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/tasks/optimizers:path_optimizer",
        "//modules/planning/tasks/optimizers/road_graph",
    ],
)

cpplint()
