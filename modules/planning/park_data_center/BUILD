load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

PLANNING_FOPENMP = [
    "-DMODULE_NAME=\\\"planning\\\"",
    "-fopenmp",
]

apollo_cc_library(
    name = "apollo_park_data_center",
    srcs = [
        "park_data_center.cc",
        "nudge_info.cc",
        "util/lane_escape_util.cc",
    ],
    hdrs = [
        "park_data_center.h",
        "nudge_info.h",
        "util/lane_escape_util.h",
    ],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    linkopts = ["-lgomp"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/util:common_util",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "@com_github_gflags_gflags//:gflags",],
    
)


apollo_package(enable_source=False)

cpplint()
