### 问题描述

在使用buildtool install planning时，只会将planning_component模块源码下载到本地，如何将planning相关所有模块全部下载？

### 解决方案

可通过通配符"*"下载planning相关所有模块源码
```shell
buildtool install planning*
```
