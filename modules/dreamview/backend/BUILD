load("//tools/platform:build_defs.bzl", "copts_if_teleop", "if_teleop")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

DREAMVIEW_COPTS = ['-DMODULE_NAME=\\"dreamview\\"']

apollo_cc_test(
    name = "simulation_world_service_test",
    size = "small",
    srcs = ["simulation_world/simulation_world_service_test.cc"],
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":apollo_dreamview_backend",
        "//modules/common/math",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_library(
    name = "apollo_dreamview_backend",
    copts = DREAMVIEW_COPTS + copts_if_teleop(),
    srcs = [
        "dreamview.cc",
        "hmi/hmi.cc",
        "hmi/hmi_worker.cc",
        "perception_camera_updater/perception_camera_updater.cc",
        "point_cloud/point_cloud_updater.cc",
        "simulation_world/simulation_world_service.cc",
        "simulation_world/simulation_world_updater.cc",
    ],
    hdrs = [
        "dreamview.h",
        "hmi/hmi.h",
        "hmi/hmi_worker.h",
        "perception_camera_updater/perception_camera_updater.h",
        "point_cloud/point_cloud_updater.h",
        "simulation_world/simulation_world_service.h",
        "simulation_world/simulation_world_updater.h",
    ],
    deps = [
        "//cyber",
        "//cyber/proto:dag_conf_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/status",
        "//modules/common/kv_db",
        "//modules/map:apollo_map",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/monitor_log",
        "//modules/transform:apollo_transform",
        "//modules/common/configs:config_gflags",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common_msgs/prediction_msgs:scenario_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common_msgs/basic_msgs:drive_event_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/localization_msgs:gps_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/storytelling_msgs:story_cc_proto",
        "//modules/common_msgs/simulation_msgs:scenario_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_status_cc_proto",
        "//modules/common_msgs/monitor_msgs:system_status_cc_proto",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/routing_msgs:poi_cc_proto",
        "//modules/common_msgs/task_manager_msgs:task_manager_cc_proto",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:valet_parking_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:action_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/common_msgs/audio_msgs:audio_cc_proto",
        "//modules/common_msgs/audio_msgs:audio_event_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/dreamview/proto:camera_update_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_config_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_mode_cc_proto",
        "//modules/common_msgs/dreamview_msgs:simulation_world_cc_proto",
        "//modules/dreamview/proto:point_cloud_cc_proto",
        "//modules/dreamview/proto:preprocess_table_cc_proto",
        "//modules/dreamview/proto:record_cc_proto",
        "//modules/dreamview/proto:scenario_cc_proto",
        "//modules/dreamview/backend/common:dreamview_common",
        "@boost",
        "@civetweb//:civetweb++",
        "@com_github_gflags_gflags//:gflags",
        "@com_github_google_glog//:glog",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_nlohmann_json//:json",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest",
        "@com_google_protobuf//:protobuf",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@opencv//:imgcodecs",
    ],
)

apollo_package()

cpplint()
