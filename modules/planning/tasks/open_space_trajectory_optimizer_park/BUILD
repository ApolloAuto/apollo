load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]
FOPENMP_COPTS = ["-fopenmp"]

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libopen_space_trajectory_optimizer_park.so",
    srcs = [
      "open_space_trajectory_optimizer_park.cc",
      "optimizer.cc",
    ],
    hdrs = [
      "open_space_trajectory_optimizer_park.h",
      "optimizer.h",
    ],
    description = ":plugins.xml",
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/planning_open_space:apollo_planning_open_space",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/open_space_trajectory_optimizer_park/proto:open_space_trajectory_optimizer_park_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_package(enable_source=False)

cpplint()
