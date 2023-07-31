# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# install_src_files(
#     name = "install_src",
#     deps = [
#         ":install_drivers_radar_src",
#         ":install_drivers_radar_hdrs"
#     ],
# )

# install_src_files(
#     name = "install_drivers_radar_src",
#     src_dir = ["."],
#     dest = "drivers-radar/src",
#     filter = "*",
# )

# install_src_files(
#     name = "install_drivers_radar_hdrs",
#     src_dir = ["."],
#     dest = "drivers-radar/include",
#     filter = "*.h",
# )

# install(
#     name = "install",
#     deps = [
#         "//modules/drivers/radar/conti_radar:install",
#         "//modules/drivers/radar/racobit_radar:install",
#         "//modules/drivers/radar/ultrasonic_radar:install",
#     ],
#     data = [
#         ":cyberfile.xml",
#         ":drivers-radar.BUILD"
#     ],
#     data_dest = "drivers-radar"
# )

apollo_package()