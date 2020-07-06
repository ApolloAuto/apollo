load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "cos_theta_smoother_config_proto",
    deps = [
        ":cos_theta_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "cos_theta_smoother_config_proto_lib",
    srcs = ["cos_theta_smoother_config.proto"],
)

cc_proto_library(
    name = "fem_pos_deviation_smoother_config_proto",
    deps = [
        ":fem_pos_deviation_smoother_config_proto_lib",
    ],
)

proto_library(
    name = "fem_pos_deviation_smoother_config_proto_lib",
    srcs = ["fem_pos_deviation_smoother_config.proto"],
)

cc_proto_library(
    name = "qp_problem_proto",
    deps = [
        ":qp_problem_lib",
    ],
)

proto_library(
    name = "qp_problem_lib",
    srcs = ["qp_problem.proto"],
)

py_proto(
    name = "cos_theta_smoother_config_pb2",
    src = "cos_theta_smoother_config.proto",
)

py_proto(
    name = "fem_pos_deviation_smoother_config_pb2",
    src = "fem_pos_deviation_smoother_config.proto",
)

py_proto(
    name = "qp_problem_pb2",
    src = "qp_problem.proto",
)
