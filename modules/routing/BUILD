load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

ROUTING_COPTS = ['-DMODULE_NAME=\\"routing\\"']

apollo_cc_library(
    name = "apollo_routing",
    srcs = [
        "common/routing_gflags.cc",
        "core/black_list_range_generator.cc",
        "core/navigator.cc",
        "core/result_generator.cc",
        "graph/node_with_range.cc",
        "graph/sub_topo_graph.cc",
        "graph/topo_graph.cc",
        "graph/topo_node.cc",
        "graph/topo_range.cc",
        "graph/topo_range_manager.cc",
        "graph/topo_test_utils.cc",
        "routing.cc",
        "strategy/a_star_strategy.cc",
        "topo_creator/edge_creator.cc",
        "topo_creator/graph_creator.cc",
        "topo_creator/node_creator.cc",
    ],
    hdrs = [
        "common/routing_gflags.h",
        "core/black_list_range_generator.h",
        "core/navigator.h",
        "core/result_generator.h",
        "graph/node_with_range.h",
        "graph/range_utils.h",
        "graph/sub_topo_graph.h",
        "graph/topo_graph.h",
        "graph/topo_node.h",
        "graph/topo_range.h",
        "graph/topo_range_manager.h",
        "graph/topo_test_utils.h",
        "routing.h",
        "strategy/a_star_strategy.h",
        "strategy/strategy.h",
        "topo_creator/edge_creator.h",
        "topo_creator/graph_creator.h",
        "topo_creator/node_creator.h",
    ],
    copts = ROUTING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/monitor_log",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/map_msgs:map_lane_cc_proto",
        "//modules/common_msgs/monitor_msgs:monitor_log_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/map:apollo_map",
        "//modules/routing/proto:routing_config_cc_proto",
        "//modules/routing/proto:topo_graph_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_component(
    name = "librouting_component.so",
    srcs = ["routing_component.cc"],
    hdrs = ["routing_component.h"],
    copts = ROUTING_COPTS,
    deps = [
        ":apollo_routing",
        "//cyber",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "conf/*.pb.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_cc_test(
    name = "graph_creator_test",
    size = "small",
    srcs = ["topo_creator/graph_creator_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":apollo_routing",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_binary(
    name = "topo_creator",
    srcs = ["topo_creator/topo_creator.cc"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":apollo_routing",
        "//modules/map:apollo_map",
    ],
)

apollo_cc_binary(
    name = "routing_dump",
    srcs = ["tools/routing_dump.cc"],
    deps = [
        ":apollo_routing",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

apollo_cc_binary(
    name = "routing_cast",
    srcs = ["tools/routing_cast.cc"],
    deps = [
        ":apollo_routing",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "tools/routing_tester/*.pb.txt",
    ]),
)

apollo_cc_binary(
    name = "routing_tester",
    srcs = ["tools/routing_tester/routing_tester.cc"],
    data = [
        ":test_data",
    ],
    deps = [
        ":apollo_routing",
    ],
)

apollo_cc_test(
    name = "topo_node_test",
    size = "small",
    srcs = ["graph/topo_node_test.cc"],
    deps = [
        ":apollo_routing",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "topo_range_test",
    size = "small",
    srcs = ["graph/topo_range_test.cc"],
    deps = [
        ":apollo_routing",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "topo_graph_test",
    size = "small",
    srcs = ["graph/topo_graph_test.cc"],
    deps = [
        ":apollo_routing",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "sub_topo_graph_test",
    size = "small",
    srcs = ["graph/sub_topo_graph_test.cc"],
    deps = [
        ":apollo_routing",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
