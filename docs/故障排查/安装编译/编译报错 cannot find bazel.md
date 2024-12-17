### 问题描述:

源码环境，用户使用sudo bash docker/scripts/dev_into.sh进入容器后执行编译，报错cannot find bazel. Please install bazel first.

![](images/cannot_find_bazel.png)

### 问题原因:

进入容器使用了sudo,导致权限混乱

### 解决方案:

执行启动容器和进入容器命令都不要添加sudo

**注:** 以当前用户执行启动容器和进入容器命令报错: dial unix /var/run/dcker.sock: connect: permission denied.需要将当前用户加入docker用户组或给docker.sock赋777权限

