package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "prediction_conf_proto",
    deps = [
        ":prediction_conf_proto_lib",
    ],
)

proto_library(
    name = "prediction_conf_proto_lib",
    srcs = [
        "prediction_conf.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
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
    srcs = [
        "prediction_obstacle.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
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
    srcs = [
        "lane_graph.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
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
    srcs = [
        "feature.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
        "//modules/prediction/proto:lane_graph_proto_lib",
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
    srcs = [
        "offline_features.proto",
    ],
    deps = [
        "//modules/prediction/proto:feature_proto_lib",
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
    srcs = [
        "fnn_model_base.proto",
    ],
    deps = [
    ],
)

cc_proto_library(
    name = "fnn_vehicle_model_proto",
    deps = [
        ":fnn_vehicle_model_proto_lib",
    ],
)

proto_library(
    name = "fnn_vehicle_model_proto_lib",
    srcs = [
        "fnn_vehicle_model.proto",
    ],
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
