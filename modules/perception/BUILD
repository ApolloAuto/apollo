load("//tools/install:install.bzl", "install", "install_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "perception_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

install(
    name = "install",
    deps = [
        ":pb_perception",
        "//modules/perception/data:install",
        "//modules/perception/onboard/component:install",
        "//modules/perception/production:install",
    ],
)

install_files(
    name = "pb_perception",
    dest = "modules/perception",
    files = [
        "//modules/perception/proto:dst_existence_fusion_config_py_pb2",
        "//modules/perception/proto:dst_type_fusion_config_py_pb2",
        "//modules/perception/proto:fused_classifier_config_py_pb2",
        "//modules/perception/proto:map_manager_config_py_pb2",
        "//modules/perception/proto:motion_service_py_pb2",
        "//modules/perception/proto:perception_camera_py_pb2",
        "//modules/perception/proto:perception_config_schema_py_pb2",
        "//modules/perception/proto:perception_lane_py_pb2",
        "//modules/perception/proto:perception_obstacle_py_pb2",
        "//modules/perception/proto:perception_ultrasonic_py_pb2",
        "//modules/perception/proto:roi_boundary_filter_config_py_pb2",
        "//modules/perception/proto:tracker_config_py_pb2",
        "//modules/perception/proto:traffic_light_detection_py_pb2",
    ],
)

cpplint()
