load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

cc_proto_library(
    name = "lgsvl_detection2d_cc_proto",
    deps = [
        ":lgsvl_detection2d_proto",
    ],
)

proto_library(
    name = "lgsvl_detection2d_proto",
    srcs = ["detection2d.proto"],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

py_proto_library(
    name = "lgsvl_detection2d_py_pb2",
    deps = [
        ":lgsvl_detection2d_proto",
    ],
)

cc_proto_library(
    name = "lgsvl_detection2darray_cc_proto",
    deps = [
        ":lgsvl_detection2darray_proto",
    ],
)

proto_library(
    name = "lgsvl_detection2darray_proto",
    srcs = ["detection2darray.proto"],
    deps = [
        ":lgsvl_detection2d_proto",
        "//modules/common/proto:header_proto",
    ],
)

py_proto_library(
    name = "lgsvl_detection2darray_py_pb2",
    deps = [
        ":lgsvl_detection2darray_proto",
    ],
)

cc_proto_library(
    name = "lgsvl_detection3d_cc_proto",
    deps = [
        ":lgsvl_detection3d_proto",
    ],
)

proto_library(
    name = "lgsvl_detection3d_proto",
    srcs = ["detection3d.proto"],
    deps = [
        ":lgsvl_detection2d_proto",
        "//modules/common/proto:geometry_proto",
        "//modules/common/proto:header_proto",
    ],
)

py_proto_library(
    name = "lgsvl_detection3d_py_pb2",
    deps = [
        ":lgsvl_detection3d_proto",
    ],
)

cc_proto_library(
    name = "lgsvl_detection3darray_cc_proto",
    deps = [
        ":lgsvl_detection3darray_proto",
    ],
)

proto_library(
    name = "lgsvl_detection3darray_proto",
    srcs = ["detection3darray.proto"],
    deps = [
        ":lgsvl_detection3d_proto",
        "//modules/common/proto:header_proto",
    ],
)

py_proto_library(
    name = "lgsvl_detection3darray_py_pb2",
    deps = [
        ":lgsvl_detection3darray_proto",
    ],
)
