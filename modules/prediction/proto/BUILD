load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "prediction_conf_proto",
    deps = [
        ":prediction_conf_proto_lib",
    ],
)

proto_library(
    name = "prediction_conf_proto_lib",
    srcs = ["prediction_conf.proto"],
    deps = [
        "//modules/perception/proto:perception_proto_lib",
        "//modules/prediction/proto:feature_proto_lib",
    ],
)

cc_proto_library(
    name = "prediction_proto",
    deps = [
        ":prediction_proto_lib",
    ],
)

proto_library(
    name = "prediction_proto_lib",
    srcs = ["prediction_obstacle.proto"],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
        "//modules/prediction/proto:feature_proto_lib",
        "//modules/prediction/proto:scenario_proto_lib",
    ],
)

cc_proto_library(
    name = "lane_graph_proto",
    deps = [
        ":lane_graph_proto_lib",
    ],
)

proto_library(
    name = "lane_graph_proto_lib",
    srcs = ["lane_graph.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/map/proto:map_proto_lib",
    ],
)

cc_proto_library(
    name = "feature_proto",
    deps = [
        ":feature_proto_lib",
    ],
)

proto_library(
    name = "feature_proto_lib",
    srcs = ["feature.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/map/proto:map_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
        "//modules/prediction/proto:lane_graph_proto_lib",
        "//modules/prediction/proto:prediction_point_proto_lib",
    ],
)

cc_proto_library(
    name = "offline_features_proto",
    deps = [
        ":offline_features_proto_lib",
    ],
)

proto_library(
    name = "offline_features_proto_lib",
    srcs = ["offline_features.proto"],
    deps = [
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/prediction/proto:feature_proto_lib",
        "//modules/prediction/proto:prediction_conf_proto_lib",
        "//modules/prediction/proto:prediction_proto_lib",
        "//modules/prediction/proto:scenario_proto_lib",
    ],
)

cc_proto_library(
    name = "fnn_model_base_proto",
    deps = [
        ":fnn_model_base_proto_lib",
    ],
)

proto_library(
    name = "fnn_model_base_proto_lib",
    srcs = ["fnn_model_base.proto"],
)

cc_proto_library(
    name = "fnn_vehicle_model_proto",
    deps = [
        ":fnn_vehicle_model_proto_lib",
    ],
)

proto_library(
    name = "fnn_vehicle_model_proto_lib",
    srcs = ["fnn_vehicle_model.proto"],
    deps = [
        "//modules/prediction/proto:fnn_model_base_proto_lib",
    ],
)

cc_proto_library(
    name = "network_model_proto",
    deps = [
        ":network_model_proto_lib",
    ],
)

proto_library(
    name = "network_model_proto_lib",
    srcs = [
        "network_layers.proto",
        "network_model.proto",
    ],
)

cc_proto_library(
    name = "scenario_proto",
    deps = [
        ":scenario_proto_lib",
    ],
)

proto_library(
    name = "scenario_proto_lib",
    srcs = ["scenario.proto"],
)

cc_proto_library(
    name = "prediction_point_proto",
    deps = [
        ":prediction_point_proto_lib",
    ],
)

proto_library(
    name = "prediction_point_proto_lib",
    srcs = ["prediction_point.proto"],
)

py_proto(
    name = "scenario_pb2",
    src = "scenario.proto",
)

py_proto(
    name = "lane_graph_pb2",
    src = "lane_graph.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/map/proto:map_lane_pb2",
    ],
)

py_proto(
    name = "feature_pb2",
    src = "feature.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/map/proto:map_lane_pb2",
        "//modules/perception/proto:perception_obstacle_pb2",
        "//modules/prediction/proto:lane_graph_pb2",
        "//modules/prediction/proto:prediction_point_pb2",
    ],
)

py_proto(
    name = "network_model_pb2",
    src = "network_model.proto",
    deps = [
        "//modules/prediction/proto:network_layers_pb2",
    ],
)

py_proto(
    name = "prediction_conf_pb2",
    src = "prediction_conf.proto",
    deps = [
        "//modules/perception/proto:perception_obstacle_pb2",
        "//modules/prediction/proto:feature_pb2",
    ],
)

py_proto(
    name = "network_layers_pb2",
    src = "network_layers.proto",
)

py_proto(
    name = "offline_features_pb2",
    src = "offline_features.proto",
    deps = [
        "//modules/common/proto:pnc_point_pb2",
        "//modules/prediction/proto:feature_pb2",
        "//modules/prediction/proto:prediction_conf_pb2",
        "//modules/prediction/proto:scenario_pb2",
    ],
)

py_proto(
    name = "prediction_obstacle_pb2",
    src = "prediction_obstacle.proto",
    deps = [
        "//modules/common/proto:error_code_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/perception/proto:perception_obstacle_pb2",
        "//modules/prediction/proto:feature_pb2",
        "//modules/prediction/proto:scenario_pb2",
    ],
)

py_proto(
    name = "fnn_model_base_pb2",
    src = "fnn_model_base.proto",
)

py_proto(
    name = "fnn_vehicle_model_pb2",
    src = "fnn_vehicle_model.proto",
    deps = [
        "//modules/prediction/proto:fnn_model_base_pb2",
    ],
)

py_proto(
    name = "prediction_point_pb2",
    src = "prediction_point.proto",
)
