load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory_smoother",
    srcs = [
        "trajectory_smoother.cc",
    ],
    hdrs = [
        "trajectory_smoother.h",
    ],
    deps = [
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
        "//modules/planning/reference_line:spiral_reference_line_smoother",
        "//modules/tools/navi_generator/backend/common:navi_generator_gflags",
        "//modules/tools/navi_generator/proto:trajectory_util_config_proto",

    ],
)

cc_library(
    name = "quad_tiles_maker",
    srcs = [
        "quad_tiles_maker.cc",
    ],
    hdrs = [
        "quad_tiles_maker.h",
    ],
    deps = [
        "//modules/common:log",
    ],
)

cc_library(
    name = "file_operator",
    srcs = [
        "file_operator.cc",
    ],
    hdrs = [
        "file_operator.h",
    ],
    deps = [
       "//modules/common:log",
       "//modules/planning/reference_line:reference_line",
       "//third_party/json",
    ],
)

cc_library(
    name = "navigation_expander",
    srcs = [
        "navigation_expander.cc",
    ],
    hdrs = [
        "navigation_expander.h",
    ],
    deps = [
        ":file_operator",
        "//modules/planning/reference_line:reference_line",
    ],
)

cc_library(
    name = "trajectory_converter",
    srcs = [
        "trajectory_converter.cc",
    ],
    hdrs = [
        "trajectory_converter.h",
    ],
    deps = [
        ":trajectory_smoother",
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/planning/reference_line:reference_line",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/reference_line:reference_line_provider",
        "//modules/tools/navi_generator/backend/common:navi_generator_gflags",
    ],
)

cc_library(
    name = "trajectory_processor",
    srcs = [
        "trajectory_processor.cc",
    ],
    hdrs = [
        "trajectory_processor.h",
    ],
    deps = [
        "//modules/tools/navi_generator/backend/database:db_operator", 
        ":trajectory_converter", 
        ":navigation_expander",
        ":file_operator",
        ":quad_tiles_maker",
        "//modules/common/util:util",
    ],
)

cc_library(
    name = "trajectory_collector",
    srcs = [
        "trajectory_collector.cc",
    ],
    hdrs = [
        "trajectory_collector.h",
    ],
    linkopts = [
        "-pthread",
        "-lconsole_bridge",
    ],
    deps = [
       "//modules/common:log",
       "//modules/common/adapters:adapter_manager",
       "//modules/tools/navi_generator/proto:trajectory_collector_proto",
        "@ros//:ros_common",
    ],
)

cc_library(
    name = "navi_gen_json_converter",
    srcs = [
        "navi_gen_json_converter.cc",
    ],
    hdrs = [
        "navi_gen_json_converter.h",
    ],
    deps = [
       "//third_party/json",
       "//modules/localization/msf/common/util:localization_msf_common_util",
    ],
)

cc_library(
    name = "navigation_matcher",
    srcs = [
        "navigation_matcher.cc",
    ],
    hdrs = [
        "navigation_matcher.h",
    ],
    deps = [
       "//modules/common:log",
       "//modules/tools/navi_generator/backend/database:db_operator",
       "//modules/localization/msf/common/util:localization_msf_common_util",
       "//modules/tools/navi_generator/backend/common:navi_generator_gflags",
       ":file_operator",
       ":quad_tiles_maker",
       ":navi_gen_json_converter",
    ],
)

cc_library(
    name = "navigation_provider",
    srcs = [
        "navigation_provider.cc",
    ],
    hdrs = [
        "navigation_provider.h",
    ],
    deps = [
        "//modules/tools/navi_generator/backend/database:db_operator",
        "//third_party/json",
        ":navigation_matcher",
        "//modules/localization/msf/common/util:localization_msf_common_util",
    ],
)

cc_library(
    name = "navigation_editor",
    srcs = [
        "navigation_editor.cc",
    ],
    hdrs = [
        "navigation_editor.h",
    ],
    deps = [
       "//modules/tools/navi_generator/backend/database:db_operator",
       ":navigation_matcher",
       ":trajectory_processor",
       "//modules/common:log",
    ],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cc_test(
    name = "trajectory_smoother_test",
    size = "small",
    srcs = [
        "trajectory_smoother_test.cc",
    ], 
    data = [        
        ":testdata",
    ],   
    deps = [
        ":trajectory_smoother",
        "@gtest//:main",
    ],
)

cc_test(
    name = "quad_tiles_maker_test",
    size = "small",
    srcs = [
        "quad_tiles_maker_test.cc",
    ],
    deps = [
        ":quad_tiles_maker",
        "@gtest//:main",
    ],
)

cc_test(
    name = "navigation_expander_test",
    size = "small",
    srcs = [
        "navigation_expander_test.cc",
    ], 
    data = [        
        ":testdata",
    ],   
    deps = [
        ":navigation_expander",
        "@gtest//:main",
    ],
)

cc_test(
    name = "trajectory_converter_test",
    size = "small",
    srcs = [
        "trajectory_converter_test.cc",
    ],
    data = [
        ":testdata",
    ],
    deps = [
        ":trajectory_converter",
        "@gtest//:main",
    ],
)

cc_test(
    name = "trajectory_collector_test",
    size = "small",
    srcs = [
        "trajectory_collector_test.cc",
    ],
    deps = [
        ":trajectory_collector",
        "@gtest//:main",
    ],
)

cpplint()
