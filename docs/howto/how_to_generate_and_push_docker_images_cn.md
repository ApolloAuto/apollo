## 如何生成和发布docker镜像
 
 本文档将演练一次生成和发布docker镜像到docker镜像数据中心的步骤。

### 生成发布版本镜像

首先，键入指令```exit```退出Docker容器环境

然后，生成一个新的Docker镜像：

```bash
bash apollo_docker.sh gen
```

如果出现下述的报错，则需要检查一下文件夹```${HOME}/.cache/apollo_release/apollo/```是否存在，镜像文件默认存放在该文件夹下。如果不存在该文件夹，则需要新创建一个。

```bash
Release directory does not exist!
```

上述指令将在发布版本的文件夹内生成一个新的Docker镜像。发布版本镜像被命名为 *release-yyyymmdd_hhmm*。同时，最近构建的镜像文件会被标记为 *release-latest*。

### 发布docker镜像
默认情况下，如果执行下述指令，则镜像会被发布到Apolloauto/apollo上的docker镜像数据中心：

```bash
bash apollo_docker.sh push
```

使用者需要将镜像发布到个人的Docker镜像数据中心，否则会出现下述报错：

```bash
denied: requested access to resource is denied.
```

可以执行下述指令解决该问题：

```bash
docker tag apolloauto/apollo:TAG_NAME YOUR_REPO:YOUR_TAGNAME
```

现在可以通过查阅[该网站](https://docs.docker.com/engine/reference/commandline/login/#options)提供的方法登录并且获取个人仓库数据。

然后发布镜像到个人仓库的Docker镜像数据中心。参考[该网站](https://ropenscilabs.github.io/r-docker-tutorial/04-Dockerhub.html)获得其他支持性信息。
