package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "velodyne_tools_proto",
    deps = [
        ":velodyne_tools_proto_lib",
    ],
)

proto_library(
    name = "velodyne_tools_proto_lib",
    srcs = [
        "velodyne_tools_conf.proto",
    ],
    deps = [ 
     "//modules/drivers/lidar_velodyne/proto:velodyne_proto_lib",    
    ]
)
