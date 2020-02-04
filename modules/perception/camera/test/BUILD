load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "camera_app_obstacle_camera_perception_test",
    size = "medium",
    srcs = ["camera_app_obstacle_camera_perception_test.cc"],
    linkopts = [
        "-lnvinfer_plugin",
    ],
    deps = [
        "//modules/perception/camera/app:obstacle_camera_perception",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference/operators:perception_inference_operators",
        "@caffe",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@opencv",
        "@tensorrt",
    ],
)

#cc_test(
#    name = "camera_app_traffic_light_camera_perception_test",
#    srcs = ["camera_app_traffic_light_camera_perception_test.cc"],
#    linkopts = [
#        "-lnvinfer_plugin",
#    ],
#    deps = [
#        "//modules/perception/camera/app:traffic_light_camera_perception",
#        "//modules/perception/inference:inference_factory_lib",
#        "//modules/perception/inference/operators:perception_inference_operators",
#        "@caffe",
#        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
#        "@com_google_googletest//:gtest_main",
#        "@com_google_protobuf//:protobuf",
#        "@opencv",
#        "@tensorrt",
#    ],
#)
cc_test(
    name = "camera_common_camera_ground_plane_test",
    size = "small",
    srcs = ["camera_common_camera_ground_plane_test.cc"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common:camera_ground_plane",
        "//modules/perception/common/io:io_util",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "camera_common_io_util",
    hdrs = [":camera_common_io_util.h"],
)

cc_test(
    name = "camera_common_data_provider_test",
    size = "small",
    srcs = ["camera_common_data_provider_test.cc"],
    linkopts = [
        "-lnvinfer_plugin",
    ],
    deps = [
        ":camera_common_io_util",
        "//modules/perception/camera/common:data_provider",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference/operators:perception_inference_operators",
        "@caffe",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@opencv",
        "@tensorrt",
    ],
)

cc_test(
    name = "camera_common_twod_threed_util_test",
    size = "small",
    srcs = ["camera_common_twod_threed_util_test.cc"],
    linkopts = [
        "-lnvinfer_plugin",
    ],
    deps = [
        "//modules/perception/camera/common:twod_threed_util",
        "//modules/perception/common/i_lib/algorithm:i_sort",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference/operators:perception_inference_operators",
        "@caffe",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@opencv",
        "@tensorrt",
    ],
)

cc_library(
    name = "camera_common_undistortion",
    srcs = ["camera_common_undistortion.cc"],
    hdrs = ["camera_common_undistortion.h"],
    deps = [
        "//modules/perception/base",
        "//third_party:boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@opencv",
    ],
)

#cc_test(
#    name = "camera_common_undistortion_handler_test",
#    srcs = ["camera_common_undistortion_handler_test.cc"],
#    linkopts = [
#        "-lnvinfer_plugin",
#    ],
#    deps = [
#        ":camera_common_io_util",
#        ":camera_common_undistortion",
#        "//modules/perception/camera/common:undistortion_handler",
#        "//modules/perception/inference:inference_factory_lib",
#        "//modules/perception/inference/operators:perception_inference_operators",
#        "//modules/perception/lib/utils:utils",
#        "@caffe",
#        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
#        "@com_google_googletest//:gtest_main",
#        "@com_google_protobuf//:protobuf",
#        "@opencv",
#        "@tensorrt",
#    ],
#)
cc_test(
    name = "camera_common_util_test",
    size = "small",
    srcs = ["camera_common_util_test.cc"],
    linkopts = [
        "-lnvinfer_plugin",
    ],
    deps = [
        ":camera_common_io_util",
        "//modules/perception/camera/common:data_provider",
        "//modules/perception/camera/common:util",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference/operators:perception_inference_operators",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "@caffe",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@opencv",
        "@tensorrt",
    ],
)

cc_test(
    name = "camera_common_veh_template_test",
    size = "small",
    srcs = ["camera_common_veh_template_test.cc"],
    linkopts = [
        "-lnvinfer_plugin",
    ],
    deps = [
        "//modules/perception/camera/common:object_template_manager",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference/operators:perception_inference_operators",
        "@caffe",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@opencv",
        "@tensorrt",
    ],
)

cc_test(
    name = "camera_lib_calibration_service_online_calibration_service_test",
    size = "small",
    srcs = ["camera_lib_calibration_service_online_calibration_service_test.cc"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/lib/calibration_service/online_calibration_service",
        "//modules/perception/camera/lib/calibrator/laneline:laneline_calibrator",
        "//modules/perception/common/io:io_util",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "camera_lib_calibrator_common_histogram_estimator_test",
    size = "small",
    srcs = ["camera_lib_calibrator_common_histogram_estimator_test.cc"],
    deps = [
        "//modules/perception/camera/lib/calibrator/common:histogram_estimator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "camera_lib_calibrator_laneline_app_util",
    srcs = ["camera_lib_calibrator_laneline_app_util.cc"],
    hdrs = ["camera_lib_calibrator_laneline_app_util.h"],
    deps = [
        "//modules/perception/common/i_lib/core",
        "//modules/perception/common/i_lib/geometry:i_plane",
        "//third_party:boost",
        "@opencv",
    ],
)

cc_library(
    name = "camera_lib_calibrator_laneline_lane_calibrator_util",
    srcs = ["camera_lib_calibrator_laneline_lane_calibrator_util.cc"],
    hdrs = ["camera_lib_calibrator_laneline_lane_calibrator_util.h"],
    deps = [
        "//modules/perception/common/i_lib/core",
        "//modules/perception/common/i_lib/geometry:i_plane",
        "@eigen",
        "@opencv",
    ],
)

cc_library(
    name = "camera_lib_calibrator_laneline_lane_io",
    srcs = ["camera_lib_calibrator_laneline_lane_io.cc"],
    hdrs = ["camera_lib_calibrator_laneline_lane_io.h"],
    deps = [
        ":camera_lib_calibrator_laneline_app_util",
        "//modules/perception/camera/lib/calibrator/laneline:lane_based_calibrator",
        "//modules/perception/common/i_lib/core",
        "@com_google_absl//absl/strings",
        "@eigen",
    ],
)

# cc_test(
#     name = "camera_lib_calibrator_laneline_laneline_calibrator_test",
#     srcs = ["camera_lib_calibrator_laneline_laneline_calibrator_test.cc"],
#     deps = [
#         ":camera_lib_calibrator_laneline_app_util",
#         ":camera_lib_calibrator_laneline_lane_calibrator_util",
#         ":camera_lib_calibrator_laneline_lane_io",
#         "//modules/perception/camera/lib/calibrator/laneline:laneline_calibrator",
#         "@com_google_googletest//:gtest_main",
#         "@opencv",
#     ],
# )

cc_test(
    name = "camera_lib_dummy_algorithms_test",
    size = "small",
    srcs = ["camera_lib_dummy_algorithms_test.cc"],
    deps = [
        "//modules/perception/camera/lib/dummy:dummy_algorithms",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "camera_lib_feature_extractor_tfe_tracking_feat_extractor_test",
    size = "small",
    srcs = ["camera_lib_feature_extractor_tfe_tracking_feat_extractor_test.cc"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/lib/feature_extractor/tfe:tracking_feat_extractor",
        "//modules/perception/common/io:io_util",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/inference/utils:inference_gemm_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "camera_lib_interface_test",
    size = "small",
    srcs = ["camera_lib_interface_test.cc"],
    deps = [
        "//modules/perception/camera/lib/interface",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "camera_lib_lane_common_functions_test",
    size = "small",
    srcs = ["camera_lib_lane_common_functions_test.cc"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/lib/lane/common:common_functions",
        "//modules/perception/common/io:io_util",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@opencv",
    ],
)

cc_test(
    name = "camera_lib_lane_detector_denseline_lane_detector_test",
    size = "medium",
    srcs = ["camera_lib_lane_detector_denseline_lane_detector_test.cc"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/lib/lane/detector/denseline:denseline_lane_detector",
        "//modules/perception/common/io:io_util",
        "@com_google_googletest//:gtest_main",
        "@opencv",
    ],
)

cc_test(
    name = "camera_lib_lane_postprocessor_denseline_lane_postprocessor_test",
    size = "small",
    srcs = ["camera_lib_lane_postprocessor_denseline_lane_postprocessor_test.cc"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/calibration_service/online_calibration_service",
        "//modules/perception/camera/lib/calibrator/laneline:laneline_calibrator",
        "//modules/perception/camera/lib/lane/detector/denseline:denseline_lane_detector",
        "//modules/perception/camera/lib/lane/postprocessor/denseline:denseline_lane_postprocessor",
        "//modules/perception/common/io:io_util",
        "@com_google_googletest//:gtest_main",
        "@opencv",
    ],
)

cc_test(
    name = "camera_lib_lane_detector_darkscnn_lane_detector_test",
    size = "medium",
    srcs = ["camera_lib_lane_detector_darkscnn_lane_detector_test.cc"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/lib/lane/detector/darkSCNN:darkSCNN_lane_detector",
        "//modules/perception/common/io:io_util",
        "@com_google_googletest//:gtest_main",
        "@opencv",
    ],
)

cc_test(
    name = "camera_lib_lane_postprocessor_darkscnn_lane_postprocessor_test",
    size = "medium",
    srcs = ["camera_lib_lane_postprocessor_darkscnn_lane_postprocessor_test.cc"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/calibration_service/online_calibration_service",
        "//modules/perception/camera/lib/calibrator/laneline:laneline_calibrator",
        "//modules/perception/camera/lib/lane/detector/darkSCNN:darkSCNN_lane_detector",
        "//modules/perception/camera/lib/lane/postprocessor/darkSCNN:darkSCNN_lane_postprocessor",
        "//modules/perception/camera/tools/offline:visualizer",
        "//modules/perception/common/io:io_util",
        "@com_google_googletest//:gtest_main",
        "@opencv",
    ],
)

cc_test(
    name = "camera_lib_obstacle_detector_yolo_region_output_test",
    size = "small",
    srcs = ["camera_lib_obstacle_detector_yolo_region_output_test.cc"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/lib/obstacle/detector/yolo:yolo_obstacle_detector",
        "//modules/perception/common/io:io_util",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/inference/utils:inference_gemm_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "camera_lib_obstacle_detector_yolo_yolo_obstacle_detector_test",
    size = "medium",
    srcs = ["camera_lib_obstacle_detector_yolo_yolo_obstacle_detector_test.cc"],
    deps = [
        "//modules/perception/camera/lib/obstacle/detector/yolo:yolo_obstacle_detector",
        "//modules/perception/common/io:io_util",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@opencv",
    ],
)

cc_test(
    name = "camera_lib_obstacle_postprocessor_location_refiner_test",
    size = "small",
    srcs = ["camera_lib_obstacle_postprocessor_location_refiner_test.cc"],
    deps = [
        "//cyber",
        "//modules/perception/camera/lib/calibration_service/online_calibration_service",
        "//modules/perception/camera/lib/calibrator/laneline:laneline_calibrator",
        "//modules/perception/camera/lib/obstacle/postprocessor/location_refiner:location_refiner_obstacle_postprocessor",
        "//modules/perception/common/io:io_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "camera_lib_obstacle_tracker_common_test",
    size = "small",
    srcs = ["camera_lib_obstacle_tracker_common_test.cc"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/obstacle/tracker/common",
        "//modules/perception/inference/operators:perception_inference_operators",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/inference/utils:inference_gemm_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_test(
    name = "camera_lib_obstacle_tracker_omt_frame_list_test",
    size = "small",
    srcs = ["camera_lib_obstacle_tracker_omt_frame_list_test.cc"],
    deps = [
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/obstacle/tracker/omt:frame_list",
        "@com_google_googletest//:gtest_main",
    ],
)

# TODO(private)
# cc_test(
#     name = "camera_lib_obstacle_tracker_omt_omt_obstacle_tracker_test",
#     srcs = ["camera_lib_obstacle_tracker_omt_omt_obstacle_tracker_test.cc"],
#     deps = [
#         "//cyber",
#         "//modules/perception/base:base",
#         "//modules/perception/camera/common:common",
#         "//modules/perception/camera/lib/interface:interface",
#         "//modules/perception/common/geometry:common",
#         "//modules/perception/common/io:io_util",
#         "@com_github_jbeder_yaml_cpp//:yaml-cpp",
#         "@com_google_absl//absl/strings",
#         "@com_google_googletest//:gtest_main",
#         "@opencv",
#     ],
# )

# TODO(private)
# cc_test(
#     name = "camera_lib_obstacle_tracker_omt_reference_test",
#     srcs = ["camera_lib_obstacle_tracker_omt_reference_test.cc"],
#     deps = [
#         "//cyber/common:file",
#         "//modules/perception/base:base",
#         "//modules/perception/camera/common:common",
#         "//modules/perception/camera/lib/obstacle/tracker/omt:frame_list",
#         "//modules/perception/camera/lib/obstacle/tracker/omt:omt_proto",
#         "//modules/perception/camera/lib/obstacle/tracker/omt:target",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cc_test(
    name = "camera_lib_obstacle_tracker_omt_target_test",
    size = "small",
    srcs = ["camera_lib_obstacle_tracker_omt_target_test.cc"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/obstacle/tracker/omt:frame_list",
        "//modules/perception/camera/lib/obstacle/tracker/omt:omt_proto",
        "//modules/perception/camera/lib/obstacle/tracker/omt:target",
        "//modules/perception/inference/utils:inference_gemm_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "camera_lib_obstacle_transformer_multicue_multicue_obstacle_transformer_test",
    size = "small",
    srcs = ["camera_lib_obstacle_transformer_multicue_multicue_obstacle_transformer_test.cc"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/lib/obstacle/detector/yolo:yolo_obstacle_detector",
        "//modules/perception/camera/lib/obstacle/transformer/multicue:multicue_obstacle_transformer",
        "//modules/perception/common/io:io_util",
        "//modules/perception/inference/utils:inference_cuda_util_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "@com_google_googletest//:gtest_main",
        "@opencv",
    ],
)

cc_test(
    name = "camera_lib_traffic_light_detector_crop_test",
    size = "small",
    srcs = ["camera_lib_traffic_light_detector_crop_test.cc"],
    linkopts = [
        "-lnvinfer_plugin",
    ],
    deps = [
        "//modules/perception/camera/lib/traffic_light/detector/detection:cropbox",
        "@caffe",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@opencv",
        "@tensorrt",
    ],
)

# cc_test(
#     name = "camera_lib_traffic_light_detector_detection_test",
#     srcs = ["camera_lib_traffic_light_detector_detection_test.cc"],
#     linkopts = [
#         "-lnvinfer_plugin",
#     ],
#     deps = [
#         "//modules/perception/camera/lib/traffic_light/detector/detection:detection",
#         "//modules/perception/inference:inference_factory_lib",
#         "//modules/perception/inference/operators:perception_inference_operators",
#         "@caffe",
#         "@com_github_jbeder_yaml_cpp//:yaml-cpp",
#         "@com_google_googletest//:gtest_main",
#         "@com_google_protobuf//:protobuf",
#         "@opencv",
#         "@tensorrt",
#     ],
# )

#cc_test(
#    name = "camera_lib_traffic_light_detector_recognition_test",
#    srcs = ["camera_lib_traffic_light_detector_recognition_test.cc"],
#    linkopts = [
#        "-lnvinfer_plugin",
#    ],
#    deps = [
#        "//modules/perception/camera/lib/traffic_light/detector/recognition:recognition",
#        "//modules/perception/inference:inference_factory_lib",
#        "//modules/perception/inference/operators:perception_inference_operators",
#        "@caffe",
#        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
#        "@com_google_googletest//:gtest_main",
#        "@com_google_protobuf//:protobuf",
#        "@opencv",
#        "@tensorrt",
#    ],
#)
cc_test(
    name = "camera_lib_traffic_light_detector_select_test",
    size = "small",
    srcs = ["camera_lib_traffic_light_detector_select_test.cc"],
    linkopts = [
        "-lnvinfer_plugin",
    ],
    deps = [
        "//modules/perception/camera/lib/traffic_light/detector/detection:select",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference/operators:perception_inference_operators",
        "@caffe",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@opencv",
        "@tensorrt",
    ],
)

# TODO(private)
#cc_test(
#    name = "camera_lib_traffic_light_preprocessor_test",
#    srcs = ["camera_lib_traffic_light_preprocessor_test.cc"],
#    linkopts = [
#        "-lnvinfer_plugin",
#    ],
#    deps = [
#        "//modules/perception/camera/lib/traffic_light/preprocessor:tl_preprocessor",
#        "//modules/perception/common/sensor_manager:sensor_manager",
#        "//modules/perception/inference:inference_factory_lib",
#        "//modules/perception/inference/operators:perception_inference_operators",
#        "@caffe",
#        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
#        "@com_google_googletest//:gtest_main",
#        "@com_google_protobuf//:protobuf",
#        "@opencv",
#        "@tensorrt",
#    ],
#)

cc_test(
    name = "camera_lib_traffic_light_tracker_test",
    size = "small",
    srcs = ["camera_lib_traffic_light_tracker_test.cc"],
    linkopts = [
        "-lnvinfer_plugin",
    ],
    deps = [
        "//cyber",
        "//modules/perception/base:distortion_model",
        "//modules/perception/camera/lib/traffic_light/tracker:semantic_decision",
        "//modules/perception/common/io:io_util",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference/operators:perception_inference_operators",
        "@caffe",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@opencv",
        "@tensorrt",
    ],
)

cc_test(
    name = "camera_app_cipv_camera_test",
    size = "small",
    srcs = ["camera_app_cipv_camera_test.cc"],
    linkopts = [
        "-lopencv_core",
        "-lnvinfer_plugin",
        "-lopencv_imgproc",
        "-lopencv_highgui",
    ],
    deps = [
        "//cyber",
        "//modules/perception/base:distortion_model",
        "//modules/perception/camera/app:cipv_camera",
        "//modules/perception/common/io:io_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
