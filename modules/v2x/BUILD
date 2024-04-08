load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_v2x",
    srcs = [
        "common/v2x_proxy_gflags.cc",
        "fusion/apps/common/trans_tools.cc",
        "fusion/configs/ft_config_manager.cc",
        "fusion/configs/fusion_tracker_gflags.cc",
        "fusion/libs/fusion/fusion.cc",
        "v2x_proxy/app/utils.cc",
        "v2x_proxy/app/v2x_proxy.cc",
        "v2x_proxy/obu_interface/grpc_interface/grpc_client.cc",
        "v2x_proxy/obu_interface/grpc_interface/grpc_server.cc",
        "v2x_proxy/obu_interface/obu_interface_grpc_impl.cc",
        "v2x_proxy/os_interface/os_interface.cc",
        "v2x_proxy/proto_adapter/proto_adapter.cc",
    ],
    hdrs = [
        "common/v2x_proxy_gflags.h",
        "fusion/apps/common/ft_definitions.h",
        "fusion/apps/common/trans_tools.h",
        "fusion/configs/ft_config_manager.h",
        "fusion/configs/fusion_tracker_gflags.h",
        "fusion/libs/common/v2x_object.h",
        "fusion/libs/fusion/fusion.h",
        "fusion/libs/fusion/km.h",
        "fusion/libs/fusion/test_tools.h",
        "v2x_proxy/app/utils.h",
        "v2x_proxy/app/v2x_proxy.h",
        "v2x_proxy/obu_interface/grpc_interface/grpc_client.h",
        "v2x_proxy/obu_interface/grpc_interface/grpc_server.h",
        "v2x_proxy/obu_interface/obu_interface_abstract_class.h",
        "v2x_proxy/obu_interface/obu_interface_grpc_impl.h",
        "v2x_proxy/os_interface/os_interface.h",
        "v2x_proxy/proto_adapter/proto_adapter.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_camera_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
        "//modules/map:apollo_map",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/v2x/proto:fusion_params_cc_proto",
        "//modules/v2x/proto:v2x_car_status_cc_proto",
        "//modules/v2x/proto:v2x_junction_cc_proto",
        "//modules/v2x/proto:v2x_obstacles_cc_proto",
        "//modules/v2x/proto:v2x_obu_rsi_cc_proto",
        "//modules/v2x/proto:v2x_obu_traffic_light_cc_proto",
        "//modules/v2x/proto:v2x_rsi_cc_proto",
        "//modules/v2x/proto:v2x_service_car_to_obu_cc_grpc",
        "//modules/v2x/proto:v2x_service_obu_to_car_cc_grpc",
        "@boost",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_grpc_grpc//:grpc++",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

apollo_component(
    name = "libv2x_fusion_component.so",
    srcs = ["fusion/apps/v2x_fusion_component.cc",],
    hdrs = ["fusion/apps/v2x_fusion_component.h",],
    deps = [":apollo_v2x"],
)

apollo_cc_test(
    name = "v2x_fusion_component_test",
    size = "small",
    srcs = ["fusion/apps/v2x_fusion_component_test.cc"],
    linkopts = [
        "-lgomp",
    ],
    deps = [
        ":DO_NOT_IMPORT_v2x_fusion_component",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "ft_config_manager_test",
    size = "small",
    srcs = ["fusion/configs/ft_config_manager_test.cc"],
    deps = [
        ":apollo_v2x",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "v2x_object_test",
    size = "small",
    srcs = ["fusion/libs/common/v2x_object_test.cc"],
    deps = [
        ":apollo_v2x",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "fusion_test",
    size = "small",
    srcs = ["fusion/libs/fusion/fusion_test.cc"],
    deps = [
        ":apollo_v2x",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "km_test",
    size = "small",
    srcs = ["fusion/libs/fusion/km_test.cc"],
    deps = [
        ":apollo_v2x",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_binary(
    name = "v2x",
    srcs = ["v2x_proxy/app/main.cc"],
    deps = [
        ":apollo_v2x",
    ],
)

apollo_cc_test(
    name = "utils_test",
    size = "small",
    timeout = "short",
    srcs = ["v2x_proxy/app/utils_test.cc"],
    deps = [
        ":apollo_v2x",
        "//cyber",
        "//modules/map:apollo_map",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "grpc_client_test",
    size = "small",
    srcs = ["v2x_proxy/obu_interface/grpc_interface/grpc_client_test.cc"],
    deps = [
        ":apollo_v2x",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "grpc_server_test",
    size = "small",
    srcs = ["v2x_proxy/obu_interface/grpc_interface/grpc_server_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_v2x",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "obu_interface_grpc_impl_test",
    size = "small",
    srcs = ["v2x_proxy/obu_interface/obu_interface_grpc_impl_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_v2x",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "os_interface_test",
    size = "small",
    timeout = "short",
    srcs = ["v2x_proxy/os_interface/os_interface_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_v2x",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "proto_adapter_test",
    size = "small",
    timeout = "short",
    srcs = ["v2x_proxy/proto_adapter/proto_adapter_test.cc"],
    deps = [
        ":apollo_v2x",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "v2x_testdata",
    srcs = glob(["fusion/test_data/*"]),
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.conf",
        "dag/*.dag",
        "data/**",
        "launch/*.launch",
        "fusion/test_data/**",
    ]),
)

apollo_package()

cpplint()
