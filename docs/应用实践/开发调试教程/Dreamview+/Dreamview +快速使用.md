Dreamview+ 是 Apollo 内置的一个 web 应用程序，为开发者提供各个自动驾驶模块的可视化输出界面，如规划路径、定位信息、底盘状态等。在自动驾驶车辆行进过程中，通过 Dreamview+ 可以对车辆硬件、各自动驾驶模块的状态进行实时监测，利用人机交互接口对车辆及各模块进行启停等控制操作，使用 PnC 监视器等调试工具定位问题，帮助开发者对自动驾驶过程有更加全面的掌控。本实验将使用 Dreamview+ 播放并分析自动驾驶离线数据包。

## 体验地址

https://apollo.baidu.com/community/course/1

## 实验目的

- 掌握 Apollo 的启动流程及原理；
- 掌握应用 Apollo 自动驾驶调试工具去分析并定位自动驾驶问题。

## 实验步骤

### 步骤一：启动 Dreamview+

1. 在终端中，执行 Dreamview+ 启动指令，执行成功后，点击菜单栏 Dreamview+ 按钮，进入 Dreamview+ 界面。
   
   ```bash
   aem bootstrap start --plus
   ```
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_cb5f4fe.png)
   
   当出现如下界面，表示 Dreamview+ 启动成功了。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_8455c10.png)
   
   点击左下角 **个人中心** > **设置** > **全局设置**，可以选择界面语言类型。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ce0ce76.png)

### 步骤二：播放自动驾驶离线数据包

1. 点击左侧菜单栏，选择 PnC 模式，操作选择 Record。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a601a39.png)

2. 在 环境资源 > 数据包 中选择 demo3.5（如果没有数据包，可以前往资源管理中心下载）。选择完毕后，点击播放按钮，就可以看到离线数据包 demo_3.5 已经被播放出来了。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_5e8a3a5.png)

### 步骤三：运行 PNC Monitor 数据监视器

界面右侧为 PNC Monitor 组件，PNC Monitor 展示了 Planning、Control 模块相关的数据曲线。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_72ffb35.png)

### 步骤四：运行 cyber_monitor 实时通道信息监视器

打开新的终端模拟器，输入并执行 cyber_monitor。

```bash
cyber_monitor
```

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7920d63.png)

键盘的左右键可以实现 Channel 的进入与退出，通过 esc 键，可以完全退出 cyber_monitor。

至此，实验结束。
