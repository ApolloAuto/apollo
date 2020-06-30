workspace(name = "apollo")

load("//tools:workspace.bzl", "apollo_repositories")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("//tools:common.bzl", "clean_dep")
# load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

apollo_repositories()

http_archive(
    name = "com_google_absl",
    sha256 = "f41868f7a938605c92936230081175d1eae87f6ea2c248f41077c8f88316f111",
    strip_prefix = "abseil-cpp-20200225.2",
    urls = ["https://github.com/abseil/abseil-cpp/archive/20200225.2.tar.gz"],
)

#
# See https://github.com/bazelbuild/bazel/issues/11406
# maybe(
#    http_archive,
#    name = "boringssl",
#    sha256 = "fb236ae74676dba515e1230aef4cc69ab265af72fc08784a6755a319dd013ca6",
#    urls = ["https://apollo-platform-system.bj.bcebos.com/archive/6.0/boringssl-83da28a68f32023fd3b95a8ae94991a07b1f6c62.tar.gz"],
# )
# ad-rss-lib
http_archive(
    name = "ad_rss_lib",
    build_file = clean_dep("//third_party:ad_rss_lib.BUILD"),
    sha256 = "10c161733a06053f79120f389d2d28208c927eb65759799fb8d7142666b61b9f",
    strip_prefix = "ad-rss-lib-1.1.0",
    urls = ["https://github.com/intel/ad-rss-lib/archive/v1.1.0.tar.gz"],
)

# CivetWeb (web server)
http_archive(
    name = "civetweb",
    build_file = clean_dep("//third_party:civetweb.BUILD"),
    sha256 = "de7d5e7a2d9551d325898c71e41d437d5f7b51e754b242af897f7be96e713a42",
    strip_prefix = "civetweb-1.11",
    urls = ["https://github.com/civetweb/civetweb/archive/v1.11.tar.gz"],
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

##jsoncpp .so for adv_plat
#new_local_repository(
#    name = "jsoncpp",
#    build_file = "third_party/jsoncpp.BUILD",
#    path = "/usr/local/apollo/jsoncpp/",
#)

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
