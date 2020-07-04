proto_library(
    name = "lgsvl_detection2d_proto_lib",
    srcs = ["detection2d.proto"],
    deps = [
	    "//modules/common/proto:header_proto_lib",
    ],
)

proto_library(
    name = "lgsvl_detection2darray_proto_lib",
    srcs = ["detection2darray.proto"],
    deps = [
        ":lgsvl_detection2d_proto_lib",
	    "//modules/common/proto:header_proto_lib",
    ],
)

proto_library(
    name = "lgsvl_detection3d_proto_lib",
    srcs = ["detection3d.proto"],
    deps = [
        ":lgsvl_detection2d_proto_lib",
	    "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
    ],
)

proto_library(
    name = "lgsvl_detection3darray_proto_lib",
    srcs = ["detection3darray.proto"],
    deps = [
        ":lgsvl_detection3d_proto_lib",
	    "//modules/common/proto:header_proto_lib",
    ],
)
