### 问题描述:

执行aem enter报错: error retrieving current directory: getcwd: cannot access parent directories: no such file or directory

![](images/start_container_mount_failed1.png)

### 问题原因:

用户删除了宿主机的挂载文件夹，并且没有执行aem remove清除旧容器，导致容器错误

### 解决方案:

执行aem remove删除历史容器后再启动新的容器

### 相关文档:

- [步骤六：删除工程](https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE5_xAE_x89_xE8_xA3_x85_xE6_x8C_x87_xE5_x8D_x97_2_xE5_x8C_x85_xE7_xAE_xA1_xE7_x90_x86_410bb1324792103828eeacd86377c551.html)
- [aem快速入门](https://apollo.baidu.com/docs/apollo/latest/md_docs_2_xE6_xA1_x86_xE6_x9E_xB6_xE8_xAE_xBE_xE8_xAE_xA1_2_xE5_x91_xBD_xE4_xBB_xA4_xE8_xA1_x8C_xE5_xB7_xA5_xE5_x85_xB7_2aem.html)
