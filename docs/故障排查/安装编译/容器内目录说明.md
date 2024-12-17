### 容器内目录说明

#### /opt/apollo

存放安装好的预编译包的(包括各种so库，头文件，随包一起发布的配置、数据文件等)

- 使用apt install apollo-neo-xx,安装的软件包都是存放到/opt/apollo中
- 使用buildtool build xx 的产出和依赖也会存放到/opt/apollo中
- 使用buildtool install xx时，会将包和依赖存放到/opt/apollo中

#### /apollo

存放配置文件，安装包的时候配置文件会在这里创建软链

- 使用aem profile use 切换配置的时候，会在/apollo下创建软链
- 使用buildtool install xx，会在/apollo下创建配置文件的软链
- 使用buildtool build xx，会在/apollo下创建配置文件的软链

#### /apollo_workspace

代码的工作空间，存放的都是自己工程的源码

- 使用buildtool install xx，会将xx源码安装到/apollo_workspace下

##### /apollo_workspace/bazel-apollo_workspace

##### /apollo_workspace/bazel-bin

##### /apollo_workspace/bazel-out

##### /apollo_workspace/bazel-testlogs

这些目录是使用bazel构建过程中产生的软链目录，用户方便获取编译产出,可参考: [bazel输出目录布局](https://bazel.build/remote/output-directories?hl=zh-cn)


