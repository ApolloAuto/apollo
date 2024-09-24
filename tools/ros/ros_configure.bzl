load("//tools:common.bzl", "basename", "dirname")
load("//tools/platform:common.bzl", 
    "execute", "make_copy_dir_rule", "make_copy_files_rule", "get_copy_dir_files")

_ROS_CODENAME = "ROS_DISTRO"
ROS_FOUND = False
CC_TPL='''
cc_library(
    name = "{}",
    hdrs = [{}],
    srcs = [{}],
    strip_include_prefix = "{}",
    visibility = ["//visibility:public"],
)
'''

ROS_TPL='''
cc_library(
    name = "ros",
    deps = [{}],
    hdrs = ["ros_distro.h"],
    data = [":generate_ros_distro_header"],
    includes = ["."],
    visibility = ["//visibility:public"],
)
'''

ROS_DISTRO_TPL='''
genrule(
    name = "generate_ros_distro_header",
    outs = ["ros_distro.h"],
    cmd = """
    if [ -d '/opt/ros' ]; then
        if [ `ls /opt/ros | wc -l` == "1" ]; then
            if [ `ls /opt/ros` == "foxy" ]; then
                echo '#define ROS_DISTRO_FOXY 1' > $@
            elif [ `ls /opt/ros` == "galactic" ]; then
                echo '#define ROS_DISTRO_GALACTIC 1' > $@ 
            elif [ `ls /opt/ros` == "humble" ]; then
                echo '#define ROS_DISTRO_HUMBLE 1' > $@
            elif [ `ls /opt/ros` == "iron" ]; then
                echo '#define ROS_DISTRO_IRON 1' > $@
            else
                echo '// Unknown ROS distribution' > $@ 
            fi
        else
            if [ "$$ROS_DISTRO" == "foxy" ]; then
                echo '#define ROS_DISTRO_FOXY 1' > $@
            elif [ "$$ROS_DISTRO" == "galactic" ]; then
                echo '#define ROS_DISTRO_GALACTIC 1' > $@
            elif [ "$$ROS_DISTRO" == "humble" ]; then
                echo '#define ROS_DISTRO_HUMBLE 1' > $@
            elif [ "$$ROS_DISTRO" == "iron" ]; then
                echo '#define ROS_DISTRO_IRON 1' > $@
            else
                echo '// Unknown ROS distribution' > $@
            fi
        fi
    else
        echo '// No ROS distribution' > $@ 
    fi
    """,
)
'''

def find_ros_dir(repository_ctx):
    codename = None
    if _ROS_CODENAME in repository_ctx.os.environ:
        codename = repository_ctx.os.environ[_ROS_CODENAME].strip()
    else:
        cmd = """ls -1 /opt/ros/ 2>/dev/null"""
        ros_codename_str = execute(
            repository_ctx,
            ["sh", "-c", cmd],
            empty_stdout_fine = True,
            ignore_error = True,
        ).stdout.strip()
        codename_list = ros_codename_str.split("\n")
        if len(codename_list) > 1:
            fail('Multiple ros distributions found, ' + 
                'please specify codename in env variable "ROS_DISTRO"')
        codename = codename_list[0]
    if codename == None:
        print("No ros2 repositories found, " + 
            "skip to import hdrs and libs.")
        return None
    ROS_FOUND = True
    return "/opt/ros/{}".format(codename)

def find_ros_workspace_dir(repository_ctx):
    repository_path = str(repository_ctx.path("")).split("/")
    ros_ws = "/".join(repository_path[: repository_path.index(".cache")] + ["ros_ws"])
    cmd = """ls -d {} 2>/dev/null""".format(ros_ws)
    if execute(repository_ctx, ["sh", "-c", cmd],
            empty_stdout_fine = True, ignore_error = True).stdout == "":
        print("No ros2 workspace found: {} it not existed, skip to import.".format(ros_ws))
        return None
    return "{}/install".format(ros_ws)

def _create_ws_ros_repository(repository_ctx):
    if not ROS_FOUND:
        return ([], [], [])
    ros_ws_dir = find_ros_workspace_dir(repository_ctx)
    ros_ws_pkg = _ros_ws_match_package(repository_ctx, ros_ws_dir)
    # ros_ws_pkg_name = [i.split("/")[-1] for i in ros_ws_pkg]

    pkg_lib_src = _ros_match_libraries(repository_ctx, ros_ws_dir, ros_ws_pkg, True)
    pkg_lib_out = {}
    for k in pkg_lib_src:
        pkg_base_path = "{}/".format(ros_ws_dir)
        pkg_lib_out[k] = ["ros_ws/lib/" + i.replace(
                pkg_base_path, "").replace("/lib/", "/") for i in pkg_lib_src[k]]
    
    outs = []
    srcs = []
    for k in pkg_lib_src:
        srcs = srcs + [i for i in pkg_lib_src[k]]
        outs = outs + [i for i in pkg_lib_out[k]]

    pkg_hdrs_dirs = ["{}/include".format(i) for i in ros_ws_pkg]
    copy_rules = [
        make_copy_files_rule(repository_ctx,
            name = "ros_ws_lib", srcs = srcs, outs = outs),
    ]
    copy_rules = copy_rules + [
        make_copy_dir_rule(
            repository_ctx,
            name = "ros_ws_{}_incl".format(i),
            src_dir = i,
            out_dir = "ros_ws/include",
        ) for i in pkg_hdrs_dirs]
    
    ws_cc_libraries = []
    ws_cc_libraries_name = []
    for index in range(len(ros_ws_pkg)):
        pkg_name = ros_ws_pkg[index].split("/")[-1]
        strip_prefix = "ros_ws/include"
        out_hdrs = get_copy_dir_files(repository_ctx,
            src_dir = pkg_hdrs_dirs[index], out_dir = "ros_ws/include")

        hdrs_bazel_format = "".join([i.strip() for i in out_hdrs])
        srcs_bazel_format = ",".join([
            '"{}"'.format(i) for i in pkg_lib_out[ros_ws_pkg[index]]])
        ws_cc_libraries.append(
            CC_TPL.format("ws_{}".format(ros_ws_pkg[index]), 
                hdrs_bazel_format, srcs_bazel_format, strip_prefix))
        ws_cc_libraries_name.append("ws_{}".format(ros_ws_pkg[index]))

    return (copy_rules, ws_cc_libraries, ws_cc_libraries_name)
        

def _create_local_ros_repository(repository_ctx):
    ros_base_dir = find_ros_dir(repository_ctx)
    incl_dir = "{}/include".format(ros_base_dir)
    lib_path = "{}/lib".format(ros_base_dir)
    _lib_path = lib_path + "/"
    _incl_dir = incl_dir + "/"

    ros_packages = _ros_match_packages(repository_ctx, ros_base_dir)

    # Copy the library and header files.
    pkg_lib_src = _ros_match_libraries(repository_ctx, ros_base_dir, ros_packages)
    pkg_lib_out = {}
    for k in pkg_lib_src:
        pkg_lib_out[k] = ["ros/lib/" + i.replace(_lib_path, "") for i in pkg_lib_src[k]]
    
    outs = []
    srcs = []
    for k in pkg_lib_src:
        srcs = srcs + [i for i in pkg_lib_src[k]]
        outs = outs + [i for i in pkg_lib_out[k]]

    out_hdrs = get_copy_dir_files(
        repository_ctx, src_dir = incl_dir, out_dir = "ros/include")
    copy_rules = [
        make_copy_files_rule(
            repository_ctx,
            name = "ros_lib",
            srcs = srcs,
            outs = outs,
        ),
        make_copy_dir_rule(
            repository_ctx,
            name = "ros_include",
            src_dir = incl_dir,
            out_dir = "ros/include",
        ),
    ]

    out_hdrs_dict = {}
    for i in out_hdrs:
        pkg_name = i.split("/")[2]
        if pkg_name not in out_hdrs_dict:
            out_hdrs_dict[pkg_name] = [i]
        else:
            out_hdrs_dict[pkg_name].append(i)

    cc_libraries = []
    cc_libraries_name = []
    for i in ros_packages:
        strip_prefix = "ros/include"
        # if not out_hdrs_dict[i][0].strip().startswith('"{}/{}'.format(strip_prefix, i)):
        #     strip_prefix = "ros/include"
        hdrs = []
        for j in out_hdrs_dict[i]:
            if j.strip().startswith('"{}/{}/{}/'.format("ros/include", i, i)):
                strip_prefix = "{}/{}".format("ros/include", i)
            hdrs.append(j.strip())
        hdrs_bazel_format = "".join(hdrs)
        srcs_bazel_format = ",".join(['"{}"'.format(l) for l in pkg_lib_out[i]])
        cc_libraries.append(
            CC_TPL.format(i, hdrs_bazel_format, srcs_bazel_format, strip_prefix))
        cc_libraries_name.append(i)
    
    return (copy_rules, cc_libraries, cc_libraries_name)

def _ros_ws_match_package(repository_ctx, ros_dir = None):
    if ros_dir == None:
        return []
    cmd = """ls -1 -d {}/*/""".format(ros_dir)
    result = execute(repository_ctx, ["sh", "-c", cmd],
            empty_stdout_fine = True, ignore_error = True).stdout.strip()
    if result == "":
        pkgs = []
    else:
        pkgs = [i[: len(i)-1] for i in result.split("\n")]

    return pkgs

def _ros_match_packages(repository_ctx, ros_dir = None):
    # ros not installed, return empty list
    if ros_dir == None:
        return []
    cmd = """ls -1 {}/include""".format(ros_dir)
    result = execute(
        repository_ctx,
        ["sh", "-c", cmd],
        empty_stdout_fine = True,
        ignore_error = True,
    ).stdout.strip()

    if result == "":
        pkgs = []
    else:
        pkgs = result.split("\n")

    incompatible_pkg_removed = []
    for i in pkgs:
        # skip rtps for avoiding symbol conflict
        if "fastrtps" in i or "fastcdr" in i \
            or "qt" in i or "rviz" in i \
            or i.endswith(".h") or i.endswith(".hpp"):
            continue
        incompatible_pkg_removed.append(i)

    return incompatible_pkg_removed

def _ros_match_libraries(repository_ctx, ros_dir = None, pkgs = [], ros_ws_match = False):
    # return empty list if ros ws / ros not existed
    libraries = {}
    if ros_dir == None:
        return libraries
    for pkg in pkgs:
        if ros_ws_match:
            cmd = """find {}/lib -name lib*.so""".format(pkg)
        else:
            cmd = """ls -1 {}/lib/lib{}*.so | grep -E --color=never 'lib{}__[a-zA-Z0-9_]*\\.so$|lib{}\\.so$' | grep -v 'connext_c'""".format(ros_dir, pkg, pkg, pkg)
        result = execute(repository_ctx, ["sh", "-c", cmd],
            empty_stdout_fine = True, ignore_error = True).stdout.strip()
        libraries[pkg] = []
        if result != "":
            result_libs = result.split("\n")
            for i in result_libs:
                if "fastrtps" in i or "fastcdr" in i:
                    continue
                libraries[pkg].append(i)
    return libraries

def _ros_configure_impl(repository_ctx):
    # Room for _create_remote_ros_repository
    (copy_rules, cc_libraries,
        cc_libraries_name) = _create_local_ros_repository(repository_ctx)
    (ws_copy_rules, ws_cc_libraries,
        ws_cc_libraries_name) = _create_ws_ros_repository(repository_ctx)

    ros_interface = ROS_TPL.format(",".join([
        '"{}"'.format(i) for i in cc_libraries_name + ws_cc_libraries_name]))

    # Set up BUILD file.
    build_tpl = repository_ctx.path(Label("//tools/ros:BUILD.tpl"))
    repository_ctx.template("BUILD", build_tpl, {
        "%{ros_distro_gen_rules}": "%s" % (ROS_DISTRO_TPL),
        "%{copy_rules}": "\n".join(copy_rules),
        "%{ws_copy_rules}": "\n".join(ws_copy_rules), 
        "%{cc_libraries}": "\n".join(cc_libraries),
        "%{ws_cc_libraries}": "\n".join(ws_cc_libraries),
        "%{ros_interface}": "%s" % (ros_interface),
    })

ros_configure = repository_rule(
    implementation = _ros_configure_impl,
    environ = [],
    local = True,
)

"""Detects and configures the local ros library.
Add the following to your WORKSPACE FILE:

```python
ros_configure(name = "local_config_ros")
```

Args:
  name: A unique name for this workspace rule.
"""
 