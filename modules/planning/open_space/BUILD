load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "node3d",
    srcs = ["node3d.cc"],
    hdrs = ["node3d.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/math",
        "//modules/planning/common:obstacle",
        "//modules/planning/constraint_checker:collision_checker",
        "//modules/planning/proto:planner_open_space_config_proto",
    ],
)

cc_library(
    name = "reeds_shepp_path",
    srcs = ["reeds_shepp_path.cc"],
    hdrs = ["reeds_shepp_path.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/open_space:node3d",
        "//modules/planning/proto:planner_open_space_config_proto",
    ],
)

cc_library(
    name = "open_space_utils",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/open_space:node3d",
        "//modules/planning/open_space:reeds_shepp_path",
    ],
)

cc_test(
    name = "reeds_shepp_path_test",
    size = "small",
    srcs = ["reeds_shepp_path_test.cc"],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/open_space:open_space_utils",
        "//modules/planning/proto:planner_open_space_config_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "node3d_test",
    size = "small",
    srcs = ["node3d_test.cc"],
    deps = [
        "//cyber/common:log",
        "//modules/planning/open_space:open_space_utils",
        "@gtest//:main",
    ],
)

cc_library(
    name = "dual_variable_warm_start_problem",
    srcs = [
        "dual_variable_warm_start_problem.cc",
    ],
    hdrs = [
        "dual_variable_warm_start_problem.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "dual_variable_warm_start_ipopt_interface",
    ],
)

cc_library(
    name = "dual_variable_warm_start_ipopt_interface",
    srcs = [
        "dual_variable_warm_start_ipopt_interface.cc",
    ],
    hdrs = [
        "dual_variable_warm_start_ipopt_interface.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/proto:planning_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "distance_approach_problem",
    srcs = [
        "distance_approach_problem.cc",
    ],
    hdrs = [
        "distance_approach_problem.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "distance_approach_ipopt_interface",
        "//cyber/common:log",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
    ],
)

cc_library(
    name = "distance_approach_ipopt_interface",
    srcs = [
        "distance_approach_ipopt_interface.cc",
    ],
    hdrs = [
        "distance_approach_ipopt_interface.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/proto:planning_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_test(
    name = "distance_approach_ipopt_interface_test",
    size = "small",
    srcs = [
        "distance_approach_ipopt_interface_test.cc",
    ],
    deps = [
        ":distance_approach_ipopt_interface",
        "//cyber/common:log",
        "@gtest//:main",
    ],
)

cc_test(
    name = "distance_approach_problem_test",
    size = "small",
    srcs = [
        "distance_approach_problem_test.cc",
    ],
    deps = [
        ":distance_approach_problem",
        "//cyber/common:log",
        "@gtest//:main",
    ],
)

cc_test(
    name = "dual_variable_warm_start_ipopt_interface_test",
    size = "small",
    srcs = [
        "dual_variable_warm_start_ipopt_interface_test.cc",
    ],
    deps = [
        ":dual_variable_warm_start_ipopt_interface",
        "@gtest//:main",
    ],
)

cc_test(
    name = "dual_variable_warm_start_problem_test",
    size = "small",
    srcs = [
        "dual_variable_warm_start_problem_test.cc",
    ],
    deps = [
        ":dual_variable_warm_start_problem",
        "@gtest//:main",
    ],
)

cc_library(
    name = "hybrid_a_star",
    srcs = [
        "hybrid_a_star.cc",
    ],
    hdrs = [
        "hybrid_a_star.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/common:obstacle",
        "//modules/planning/open_space:open_space_utils",
        "//modules/planning/proto:planner_open_space_config_proto",
    ],
)

cc_test(
    name = "hybrid_a_star_test",
    size = "small",
    srcs = [
        "hybrid_a_star_test.cc",
    ],
    deps = [
        "//modules/planning/open_space:hybrid_a_star",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "hybrid_a_star_wrapper_lib.so",
    srcs = [
        "hybrid_a_star_wrapper.cc",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//modules/planning/open_space:hybrid_a_star",
    ],
)

cc_binary(
    name = "distance_approach_problem_wrapper_lib.so",
    srcs = [
        "distance_approach_problem_wrapper.cc",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//modules/planning/open_space:distance_approach_problem",
        "//modules/planning/open_space:dual_variable_warm_start_problem",
        "//modules/planning/open_space:hybrid_a_star",
    ],
)

cc_library(
    name = "open_space_trajectory_generator",
    srcs = [
        "open_space_trajectory_generator.cc",
    ],
    hdrs = [
        "open_space_trajectory_generator.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "distance_approach_problem",
        "hybrid_a_star",
        "dual_variable_warm_start_problem",
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_common",
        "//modules/planning/proto:planning_proto",
        "@eigen//:eigen",
    ],
)

cc_binary(
    name = "open_space_roi_wrapper_lib.so",
    linkshared = True,
    linkstatic = False,
    srcs = [
        "open_space_roi_wrapper.cc",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/math",
        "//modules/map/pnc_map",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "open_space_ROI",
    srcs = [
        "open_space_ROI.cc",
    ],
    hdrs = [
        "open_space_ROI.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state/proto:vehicle_state_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/map/pnc_map",
        "//modules/planning/common:frame",
        "//modules/planning/common:indexed_queue",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
        "@eigen",
    ],
)

cpplint()
