workspace(name = "apollo")

load("//tools:workspace.bzl", "apollo_repositories")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("//third_party/gpus:cuda_configure.bzl", "cuda_configure")
load("//third_party/tensorrt:tensorrt_configure.bzl", "tensorrt_configure")
load("//third_party/py:python_configure.bzl", "python_configure")
load("//third_party/vtk:vtk_configure.bzl", "vtk_configure")
load("//tools:common.bzl", "clean_dep")
# load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

cuda_configure(name = "local_config_cuda")

tensorrt_configure(name = "local_config_tensorrt")

python_configure(name = "local_config_python")

vtk_configure(name = "local_config_vtk")

apollo_repositories()

http_archive(
    name = "bazel_skylib",
    sha256 = "1dde365491125a3db70731e25658dfdd3bc5dbdfd11b840b3e987ecf043c7ca0",
    urls = ["https://github.com/bazelbuild/bazel-skylib/releases/download/0.9.0/bazel_skylib-0.9.0.tar.gz"],
)

load("@bazel_skylib//:workspace.bzl", "bazel_skylib_workspace")

bazel_skylib_workspace()

# googletest (GTest and GMock)
http_archive(
    name = "com_google_googletest",
    sha256 = "9dc9157a9a1551ec7a7e43daea9a694a0bb5fb8bec81235d8a1e6ef64c716dcb",
    strip_prefix = "googletest-release-1.10.0",
    url = "https://github.com/google/googletest/archive/release-1.10.0.tar.gz",
)

http_archive(
    name = "com_google_protobuf",
    sha256 = "71030a04aedf9f612d2991c1c552317038c3c5a2b578ac4745267a45e7037c29",
    strip_prefix = "protobuf-3.12.3",
    urls = ["https://github.com/protocolbuffers/protobuf/archive/v3.12.3.tar.gz"],
)

http_archive(
    name = "rules_proto",
    sha256 = "602e7161d9195e50246177e7c55b2f39950a9cf7366f74ed5f22fd45750cd208",
    strip_prefix = "rules_proto-97d8af4dc474595af3900dd85cb3a29ad28cc313",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_proto/archive/97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz",
        "https://github.com/bazelbuild/rules_proto/archive/97d8af4dc474595af3900dd85cb3a29ad28cc313.tar.gz",
    ],
)

load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")

rules_proto_dependencies()

rules_proto_toolchains()

http_archive(
    name = "com_google_absl",
    sha256 = "f41868f7a938605c92936230081175d1eae87f6ea2c248f41077c8f88316f111",
    strip_prefix = "abseil-cpp-20200225.2",
    urls = ["https://github.com/abseil/abseil-cpp/archive/20200225.2.tar.gz"],
)

new_local_repository(
    name = "poco",
    build_file = "third_party/poco.BUILD",
    path = "/opt/apollo/sysroot/include",
)

#http_archive(
#    name = "com_github_gflags_gflags",
#    build_file = "gflags.BUILD",
#    sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
#    strip_prefix = "gflags-2.2.2",
#    urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
#)
#
#http_archive(
#    name = "com_google_glog",
#    build_file = "glog.BUILD",
#    sha256 = "f28359aeba12f30d73d9e4711ef356dc842886968112162bc73002645139c39c",
#    strip_prefix = "glog-0.4.0",
#    url = "https://github.com/google/glog/archive/v0.4.0.tar.gz",
#)
# See https://github.com/bazelbuild/bazel/issues/11406
# maybe(
#    http_archive,
#    name = "boringssl",
#    sha256 = "fb236ae74676dba515e1230aef4cc69ab265af72fc08784a6755a319dd013ca6",
#    urls = ["https://apollo-platform-system.bj.bcebos.com/archive/6.0/boringssl-83da28a68f32023fd3b95a8ae94991a07b1f6c62.tar.gz"],
# )
# grpc
http_archive(
    name = "com_github_grpc_grpc",
    sha256 = "419dba362eaf8f1d36849ceee17c3e2ff8ff12ac666b42d3ff02a164ebe090e9",
    strip_prefix = "grpc-1.30.0",
    urls = ["https://github.com/grpc/grpc/archive/v1.30.0.tar.gz"],
)

load("@com_github_grpc_grpc//bazel:grpc_deps.bzl", "grpc_deps")

grpc_deps()

load("@com_github_grpc_grpc//bazel:grpc_extra_deps.bzl", "grpc_extra_deps")

grpc_extra_deps()

# Cpplint
http_archive(
    name = "cpplint",
    build_file = clean_dep("//third_party:cpplint.BUILD"),
    sha256 = "96db293564624543a2fd3b1a0d23f663b8054c79853a5918523655721a9f6b53",
    strip_prefix = "cpplint-1.4.5",
    urls = ["https://github.com/cpplint/cpplint/archive/1.4.5.tar.gz"],
)

# ad-rss-lib
http_archive(
    name = "ad_rss_lib",
    build_file = clean_dep("//third_party:ad_rss_lib.BUILD"),
    sha256 = "10c161733a06053f79120f389d2d28208c927eb65759799fb8d7142666b61b9f",
    strip_prefix = "ad-rss-lib-1.1.0",
    urls = ["https://github.com/intel/ad-rss-lib/archive/v1.1.0.tar.gz"],
)

# eigen
http_archive(
    name = "eigen",
    build_file = clean_dep("//third_party:eigen.BUILD"),
    sha256 = "a8d87c8df67b0404e97bcef37faf3b140ba467bc060e2b883192165b319cea8d",
    strip_prefix = "eigen-git-mirror-3.3.7",
    urls = ["https://github.com/eigenteam/eigen-git-mirror/archive/3.3.7.tar.gz"],
)

# CivetWeb (web server)
http_archive(
    name = "civetweb",
    build_file = clean_dep("//third_party:civetweb.BUILD"),
    sha256 = "de7d5e7a2d9551d325898c71e41d437d5f7b51e754b242af897f7be96e713a42",
    strip_prefix = "civetweb-1.11",
    urls = ["https://github.com/civetweb/civetweb/archive/v1.11.tar.gz"],
)

#ros
#new_local_repository(
#    name = "ros",
#    build_file = "third_party/ros.BUILD",
#    path = "/home/tmp/ros",
#)
#
# PCL 1.11
new_local_repository(
    name = "pcl",
    build_file = "third_party/pcl.BUILD",
    path = "/opt/apollo/sysroot/include/pcl-1.10",
)

new_local_repository(
    name = "glew",
    build_file = "third_party/glew.BUILD",
    path = "/usr/include",
)

new_local_repository(
    name = "qt",
    build_file = "third_party/qt5/qt.BUILD",
    path = "/usr/local/qt5/include",
)

#new_local_repository(
#    name = "opengl",
#    build_file = "third_party/opengl.BUILD",
#    path = "/usr/include",
#)
#new_local_repository(
#    name = "glfw",
#    build_file = "third_party/glfw.BUILD",
#    path = "/usr/include",
#)

# Caffe
# new_local_repository(
#    name = "caffe",
#    build_file = "third_party/caffe.BUILD",
#    path = "/opt/apollo/pkgs/caffe/include",
# )

# YAML-CPP
http_archive(
    name = "com_github_jbeder_yaml_cpp",
    build_file = clean_dep("//third_party:yaml_cpp.BUILD"),
    sha256 = "77ea1b90b3718aa0c324207cb29418f5bced2354c2e483a9523d98c3460af1ed",
    strip_prefix = "yaml-cpp-yaml-cpp-0.6.3",
    urls = ["https://github.com/jbeder/yaml-cpp/archive/yaml-cpp-0.6.3.tar.gz"],
)

new_local_repository(
    name = "qpOASES",
    build_file = "third_party/qpOASES.BUILD",
    path = "/opt/apollo/sysroot/include",
)

# OSQP
new_local_repository(
    name = "osqp",
    build_file = "third_party/osqp.BUILD",
    path = "/opt/apollo/sysroot/include",
)

# ipopt
new_local_repository(
    name = "ipopt",
    build_file = "third_party/ipopt.BUILD",
    path = "/usr/include",
)

# ADOL-C
new_local_repository(
    name = "adolc",
    build_file = "third_party/adolc.BUILD",
    path = "/usr/include",
)

## Local-integ
new_local_repository(
    name = "local_integ",
    build_file = "third_party/local_integ.BUILD",
    path = "/usr/local/apollo/local_integ",
)

## mkldnn
#new_local_repository(
#    name = "mkldnn",
#    build_file = "third_party/mkldnn.BUILD",
#    path = "/usr/local/apollo/local_third_party/mkldnn",
#)
#

# mklml
# new_local_repository(
#    name = "mklml",
#    build_file = "third_party/mklml.BUILD",
#    path = "/usr/local/apollo/local_third_party/mklml",
# )

## Proj.4
new_local_repository(
    name = "proj4",
    build_file = "third_party/proj4.BUILD",
    path = "/usr/include",
)

new_local_repository(
    name = "tinyxml2",
    build_file = "third_party/tinyxml2.BUILD",
    path = "/usr/include",
)

##jsoncpp .so for adv_plat
#new_local_repository(
#    name = "jsoncpp",
#    build_file = "third_party/jsoncpp.BUILD",
#    path = "/usr/local/apollo/jsoncpp/",
#)

#adv_plat
new_local_repository(
    name = "adv_plat",
    build_file = "third_party/adv_plat.BUILD",
    path = "/opt/apollo/pkgs/adv_plat/include",
)

new_local_repository(
    name = "fastrtps",
    build_file = "third_party/fastrtps.BUILD",
    path = "/usr/local/fast-rtps",
)

## libtorch
new_local_repository(
    name = "libtorch_cpu",
    build_file = "third_party/libtorch_cpu.BUILD",
    path = "/usr/local/libtorch_cpu/include",
)

# libtorch GPU
new_local_repository(
    name = "libtorch_gpu",
    build_file = "third_party/libtorch_gpu.BUILD",
    path = "/usr/local/libtorch_gpu/include",
)

new_local_repository(
    name = "opencv",
    build_file = "third_party/opencv.BUILD",
    path = "/usr/include",
)

# Needed by modules/transform
new_local_repository(
    name = "tf2",
    build_file = "third_party/tf2.BUILD",
    path = "/opt/apollo/pkgs/tf2/include",
)

new_local_repository(
    name = "ffmpeg",
    build_file = "third_party/ffmpeg.BUILD",
    path = "/opt/apollo/sysroot/include",
)

# lz4
new_local_repository(
    name = "lz4",
    build_file = "third_party/lz4.BUILD",
    path = "/usr/include/",
)

new_local_repository(
    name = "npp",
    build_file = "third_party/npp.BUILD",
    path = "/usr/local/cuda",
)
