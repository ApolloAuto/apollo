### 问题描述:

编译模块后，在/apollo_workspace/modules下没有找到cyberfile.xml中依赖模块的源码

### 问题原因:

cyberfile文件中指定依赖有两种方式：binary和src，binary是二进制的方式引入依赖，src是源码的方式引入依赖

### 解决方案:

修改cberfile.xml中依赖方式，将binary修改为src

![](images/cyberfile_import.png)
