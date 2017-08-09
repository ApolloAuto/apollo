package(default_visibility = ["//visibility:public"])

# gets the filepath to protobuf
genquery(
    name = "protobuf-root",
    expression = "@com_google_protobuf//:protobuf",
    scope = ["@com_google_protobuf//:protobuf"],
    opts = ["--output=location"],
)

genrule(
    name = "configure",
    message = "Building Caffe (this may take a while)",
    srcs = [
        ":protobuf-root",
        ":CMakeLists.txt",
        ":cmake/Dependencies.cmake",
        "@com_google_protobuf//:protoc",
        "@com_google_protobuf//:protobuf",
        "//external:gflags",
        "@glog//:glog",
    ],
    outs = [
        "lib/libcaffe.so",
        "lib/libproto.a",
        "include/caffe/proto/caffe.pb.h",
    ],
    cmd =
        '''
        sed -i -e "s/Boost 1.55/Boost 1.54/g" $(location :cmake/Dependencies.cmake);
        srcdir=$$(pwd);
        workdir=$$(mktemp -d -t tmp.XXXXXXXXXX);
        outdir=$$srcdir/$(@D);

        protobuf_incl=$$(grep -oP "^/\\\S*(?=/)" $(location :protobuf-root))/src;
        protoc=$$srcdir/$(location @com_google_protobuf//:protoc);
        protolib=$$srcdir/$$(echo "$(locations @com_google_protobuf//:protobuf)" | grep -o "\\\S*/libprotobuf.so"); 
        
        gflags_lib=$$srcdir/$$(echo "$(locations //external:gflags)" | grep -o "\\\S*/libgflags.so"); 
        gflags_incl=$$(echo $$gflags_lib | sed -e"s#bin/#genfiles/#g" | grep -o "\\\S*/com_github_gflags_gflags");
 
        glog_lib=$$srcdir/$$(echo "$(locations @glog//:glog)" | grep -o "\\\S*/libglog.so"); 
        glog_incl=$$(echo $$glog_lib | sed -e"s#bin/#genfiles/#g" | grep -o "\\\S*/glog");

        ''' +

        # configure cmake.
        '''
        pushd $$workdir;
        cmake $$srcdir/$$(dirname $(location :CMakeLists.txt)) \
            -DCMAKE_INSTALL_PREFIX=$$srcdir/$(@D) \
            -DCMAKE_BUILD_TYPE=Release            \
            -DCPU_ONLY=ON                         \
            -DBUILD_python=OFF                    \
            -DBUILD_python_layer=OFF              \
            -DUSE_OPENCV=ON                       \
            -DBUILD_SHARED_LIBS=ON                \
            -DUSE_LEVELDB=ON                      \
            -DUSE_LMDB=ON                         \
            -DGFLAGS_INCLUDE_DIR=$$gflags_incl    \
            -DGFLAGS_LIBRARY=$$gflags_lib         \
            -DGLOG_INCLUDE_DIR=$$glog_incl        \
            -DGLOG_LIBRARY=$$glog_lib             \
            -DPROTOBUF_INCLUDE_DIR=$$protobuf_incl\
            -DPROTOBUF_PROTOC_EXECUTABLE=$$protoc \
            -DPROTOBUF_LIBRARY=$$protolib; ''' +

        '''
        cmake --build . --target caffe -- -j 8
        cp -r ./lib $$outdir
        cp -r ./include $$outdir; ''' +

        '''
        # clean up
        popd;
        # rm -rf $$workdir;
        '''
)

cc_library(
    name = "lib",
    includes = ["include/"],
    srcs = [
        "lib/libcaffe.so",
        "lib/libproto.a"
    ],
    hdrs = glob(["include/**"])
           + ["include/caffe/proto/caffe.pb.h"],
    deps = [
        "@com_google_protobuf//:protobuf",
        "//external:gflags",
        "@glog//:glog",
    ],
    defines = ["CPU_ONLY"],
    linkopts = [
        "-L/usr/lib/x86_64-linux-gnu/hdf5/serial/lib",
        "-Wl,-rpath,/usr/local/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial/lib",
        "-lboost_system",
        "-lboost_thread",
        "-lboost_filesystem",
        "-lpthread",
        "-lblas",
        "-lcblas",
        "-lhdf5_hl",
        "-lhdf5",
        "-lz",
        "-ldl",
        "-lm",
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    visibility = ["//visibility:public"],
    linkstatic = 0,
)
