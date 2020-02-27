def py_proto(name, src, with_grpc=False, deps=[]):
    proto_path = "$(location {})".format(src)
    pb2_file = src.replace(".proto", "_pb2.py")
    pb2_path = "$$(dirname {})/{}".format(proto_path, pb2_file)

    # Require `pip install grpcio-tools` to have the tool available.
    executable = "/usr/bin/python -m grpc_tools.protoc -I. --python_out=."
    native.genrule(
        name = name + "_rule",
        srcs = [src],
        outs = [pb2_file],
        cmd = "{} $< && mv {} $@".format(executable, pb2_path),
    )

    py_srcs = [pb2_file]
    if with_grpc:
        pb2_grpc_file = src.replace(".proto", "_pb2_grpc.py")
        pb2_grpc_path = "$$(dirname {})/{}".format(proto_path, pb2_grpc_file)
        native.genrule(
            name = name + "_grpc_rule",
            srcs = [src],
            outs = [pb2_grpc_file],
            cmd = "{} --grpc_python_out=. $< && mv {} $@".format(executable, pb2_grpc_path),
        )
        py_srcs.append(pb2_grpc_file)

    native.py_library(
        name = name,
        srcs = py_srcs,
        deps = deps,
    )
