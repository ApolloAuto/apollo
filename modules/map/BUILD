load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

apollo_cc_library(
    name = "apollo_map",
    srcs = [
        "hdmap/adapter/opendrive_adapter.cc",
        "hdmap/adapter/proto_organizer.cc",
        "hdmap/adapter/xml_parser/coordinate_convert_tool.cc",
        "hdmap/adapter/xml_parser/header_xml_parser.cc",
        "hdmap/adapter/xml_parser/junctions_xml_parser.cc",
        "hdmap/adapter/xml_parser/lanes_xml_parser.cc",
        "hdmap/adapter/xml_parser/objects_xml_parser.cc",
        "hdmap/adapter/xml_parser/roads_xml_parser.cc",
        "hdmap/adapter/xml_parser/signals_xml_parser.cc",
        "hdmap/adapter/xml_parser/util_xml_parser.cc",
        "hdmap/hdmap.cc",
        "hdmap/hdmap_common.cc",
        "hdmap/hdmap_impl.cc",
        "hdmap/hdmap_util.cc",
        "pnc_map/path.cc",
        "pnc_map/pnc_map_base.cc",
        "pnc_map/route_segments.cc",
        "relative_map/common/relative_map_gflags.cc",
        "relative_map/navigation_lane.cc",
        "relative_map/relative_map.cc",
    ],
    hdrs = [
        "hdmap/adapter/opendrive_adapter.h",
        "hdmap/adapter/proto_organizer.h",
        "hdmap/adapter/xml_parser/common_define.h",
        "hdmap/adapter/xml_parser/coordinate_convert_tool.h",
        "hdmap/adapter/xml_parser/header_xml_parser.h",
        "hdmap/adapter/xml_parser/junctions_xml_parser.h",
        "hdmap/adapter/xml_parser/lanes_xml_parser.h",
        "hdmap/adapter/xml_parser/objects_xml_parser.h",
        "hdmap/adapter/xml_parser/roads_xml_parser.h",
        "hdmap/adapter/xml_parser/signals_xml_parser.h",
        "hdmap/adapter/xml_parser/status.h",
        "hdmap/adapter/xml_parser/util_xml_parser.h",
        "hdmap/hdmap.h",
        "hdmap/hdmap_common.h",
        "hdmap/hdmap_impl.h",
        "hdmap/hdmap_util.h",
        "pnc_map/path.h",
        "pnc_map/pnc_map_base.h",
        "pnc_map/route_segments.h",
        "relative_map/common/relative_map_gflags.h",
        "relative_map/navigation_lane.h",
        "relative_map/relative_map.h",
    ],
    copts = ["-DMODULE_NAME=\\\"map\\\""],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:config_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/common_msgs/map_msgs:map_id_cc_proto",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common_msgs/map_msgs:map_area_proto",
        "//modules/common_msgs/map_msgs:map_barrier_gate_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/map/relative_map/proto:relative_map_config_cc_proto",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_github_grpc_grpc//:grpc++",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_absl//:absl",
        "@eigen",
        "@proj",
        "@tinyxml2",
    ],
)

filegroup(
    name = "map_testdata",
    srcs = glob([
        "testdata/**/*",
    ]),
)

# TODO: configurable for packing
filegroup(
    name = "map_data",
    srcs = glob([
        "data/demo/**",
        "data/borregas_ave/**",
    ]),
)

# only test use
# cuda_library(
#     name = "cuda_pnc_util",
#     srcs = ["pnc_map/cuda_util.cu"],
#     hdrs = ["pnc_map/cuda_util.h"],
#     tags = ["exclude"],
#     deps = [
#         "//cyber",
#         "//modules/common/math",
#         "@local_config_cuda//cuda:cudart",
#     ],
# )

# apollo_cc_test(
#     name = "cuda_util_test",
#     size = "small",
#     srcs = [
#         "pnc_map/cuda_util.h",
#         "pnc_map/cuda_util_test.cc",
#         ":cuda_pnc_util",
#     ],
#     linkstatic = True,
#     tags = ["exclude"],
#     deps = [
#         "//modules/common/math",
#         "@com_google_googletest//:gtest_main",
#         "@local_config_cuda//cuda:cublas",
#         "@local_config_cuda//cuda:cuda_headers",
#         "@local_config_cuda//cuda:cudart",
#     ],
# )

apollo_cc_test(
    name = "pnc_path_test",
    size = "small",
    srcs = ["pnc_map/path_test.cc"],
    deps = [
        ":apollo_map",
        "//modules/common/util:common_util",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

# comment out temporarily
# apollo_cc_test(
#     name = "route_segments_test",
#     size = "small",
#     srcs = ["pnc_map/route_segments_test.cc"],
#     data = [
#         ":pnc_testdata",
#         "//modules/map:map_data",
#     ],
#     deps = [
#         ":apollo_map",
#         "//modules/common/util:common_util",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

apollo_cc_test(
    name = "hdmap_map_test",
    size = "small",
    timeout = "short",
    srcs = [
        "hdmap/hdmap_common_test.cc",
        "hdmap/hdmap_impl_test.cc",
    ],
    data = [
        ":hd_testdata",
    ],
    tags = ["exclude"],
    deps = [
        ":apollo_map",
        "//cyber",
        "@com_github_google_glog//:glog",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "hdmap_util_test",
    size = "small",
    timeout = "short",
    srcs = ["hdmap/hdmap_util_test.cc"],
    data = [
        ":hd_testdata",
    ],
    linkstatic = True,
    deps = [
        ":apollo_map",
        "@com_github_google_glog//:glog",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "relative_map_conf",
    srcs = glob([
        "relative_map/conf/**",
        "relative_map/dag/*.dag",
        "relative_map/launch/*.launch",
    ]),
)

filegroup(
    name = "rel_testdata",
    srcs = glob([
        "relative_map/testdata/**",
    ]),
)

filegroup(
    name = "hd_testdata",
    srcs = glob([
        "hdmap/test-data/*",
    ]),
)

filegroup(
    name = "pnc_testdata",
    srcs = glob([
        "pnc_map/testdata/**",
    ]),
)

apollo_component(
    name = "librelative_map_component.so",
    srcs = ["relative_map/relative_map_component.cc"],
    hdrs = ["relative_map/relative_map_component.h"],
    deps = [":apollo_map"],
)

apollo_cc_test(
    name = "navigation_lane_test",
    size = "small",
    srcs = ["relative_map/navigation_lane_test.cc"],
    data = [
        ":rel_testdata",
        ":relative_map_conf",
    ],
    deps = [
        ":apollo_map",
        "@com_github_nlohmann_json//:json",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
