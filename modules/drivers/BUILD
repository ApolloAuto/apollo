load("//tools/install:install.bzl", "install", "install_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    deps = [
        ":pb_drivers",
        "//modules/drivers/camera:install",
        "//modules/drivers/gnss:install",
        "//modules/drivers/lidar:install",
        "//modules/drivers/microphone:install",
        "//modules/drivers/radar:install",
        "//modules/drivers/smartereye:install",
        "//modules/drivers/tools/image_decompress:install",
        "//modules/drivers/video:install",
    ],
)

install_files(
    name = "pb_drivers",
    dest = "modules/drivers",
    files = [
        "//modules/drivers/canbus/proto:can_card_parameter_py_pb2",
        "//modules/drivers/gnss/proto:ins_py_pb2",
        "//modules/drivers/proto:conti_radar_py_pb2",
        "//modules/drivers/proto:pointcloud_py_pb2",
        "//modules/drivers/proto:sensor_image_py_pb2",
        "//modules/drivers/radar/conti_radar/proto:conti_radar_conf_py_pb2",
    ],
)
