# planning

## 介绍

`planning-base` 是planning模块的基础数据结构和算法库，它包含了模块运行流程中公共的数据结构类，以及底层算法函数。

## 目录结构

```shell

modules/planning/planning_base
├── planning_base
    ├── common                  // 公共算法库
    ├── gflags                  // gflag参数配置
    ├── learning_based          // 基于学习算法相关库
    ├── math                    // 基础数学库
    ├── open_space              // open_space相关算法库
    ├── proto                   // 公共（全局）参数配置结构定义
    ├── reference_line          // 参考线以及参考线处理类
    ├── testdata                // 单元测试数据
    ├── tools                   // 工具类
    ├── BUILD                   // 构建规则文件
    ├── cyberfile.xml           // 包管理配置文件
    └── README_cn.md            // 说明文档
```

