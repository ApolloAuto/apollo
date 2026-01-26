该用户手册旨在帮助用户完成摄像头感知设备（Camera）的安装、集成及数据验证。

## 前提条件

完成了 [循迹搭建--车辆循迹演示](../应用实践/车辆集成教程/循迹搭建/车辆循迹演示.md)。

## 感知设备集成

### 1. 安装固定相机

相机应水平安装，俯仰角向下 0-2 度（向下倾斜小于 2 度，不能上仰），翻滚角误差 ±1 度（左右两侧的平齐程度），航向角误差 ±2 度，镜头保持清洁，避免影响图像采集。

### 2. 配置摄像头规则文件

摄像头规则文件的作用是，当 Linux 启动时，根据规则文件设置的规则，自动生成对应的软链接文件。

1. 在 docker 环境外，执行如下命令，创建并编辑相机规则文件：
   
   ```
   sudo vim /etc/udev/rules.d/99-webcam.rules
   ```

2. 根据自身情况，修改 rules 文件，摄像头的 rules 文件示例如下（这里只包含了 2 个摄像头的规则文件，如需用到 3 个摄像头，可参照格式自行添加)：
   
   ```
   SUBSYSTEM=="video4linux", SUBSYSTEMS=="pci", ATTR{name}=="FPD1-0100", ATTR{index}=="0", MODE="0666", SYMLINK+="camera/front_6mm", OWNER="apollo", GROUP="apollo"
   SUBSYSTEM=="video4linux", SUBSYSTEMS=="pci", ATTR{name}=="FPD3-0300", ATTR{index}=="0", MODE="0666", SYMLINK+="camera/front_12mm", OWNER="apollo", GROUP="apollo"
   ```
   
   规则文件中的 `ATTR{name}` 值请您根据实际情况进行修改。

3. 查看摄像头所接的 USB 属性信息。在 docker 环境外执行如下命令：
   
   ```
   udevadm info --attribute-walk --name=/dev/video*
   ```

4. 在 docker 环境外，执行如下命令，使配置的规则文件在本地系统生效:
   
   ```
   bash ~/apollo/docker/setup_host/setup_host.sh
   sudo reboot  //重启工控机
   ```

### 3. 启动摄像头

#### 1. 检查摄像头是否被系统识别。

在摄像头与工控机正常连接的基础上，执行 `ls /dev/video*` 指令，查看摄像头是否被识别。

- 如果摄像头设备已经被识别，则会显示以 `video` 开头的设备名称。

- 如果摄像头设备没有被识别，请检查摄像头与工控机的连线是否可靠。

#### 2. 检查是否存在摄像头软连接文件。

检查 `/dev/camera` 目录是否存在，以及该目录下 `front_6mm` 、 `front_12mm` 两个软链接文件是否存在。

> 注意：根据规则文件配置不同，可能有 1 个或多个软链接文件。

如果使用 `ls /dev/video*` 命令能显示摄像头设备，但不存在软链接文件或者没有 `camera` 文件夹，请参照上文 `摄像头规则文件的配置` 章节，检查规则文件是否配置正确。

#### 3. 编译项目，启动 Dreamview+。

##### a. 启动 Apollo Docker 环境

如果您使用的是包管理，请您按照以下步骤操作：

1. 打开命令行终端，切换路径到 Apollo：
   
   ```bash
   cd ~/apollo
   ```

2. 输入以下命令进入 Apollo Docker 环境：
   
   ```bash
   aem start #创建容器
   aem enter #进入容器
   buildtool build
   ```
   
   > 注意：首次进入容器用 `buildtool build` ；如果是只编译更新的模块，使用以下命令 `buildtool build -p 模块名` 。
   
   如果您的终端如下面所示，说明您成功进入了 Apollo 的运行容器：
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_7e7a575.png)

如果您使用的是源码，请您按照以下步骤进入 docker 环境：

1. 打开命令行终端，切换路径到 Apollo：
   
   ```bash
   cd ~/apollo     #切换路径到apollo，注意cd与~/apollo之间有一个空格
   ```

2. 输入以下命令启动 Apollo Docker 环境：
   
   ```bash
   bash docker/scripts/dev_start.sh
   ```
   
   启动 docker 环境：
   
   ```bash
   bash docker/scripts/dev_into.sh
   ```
   
   如果您的终端如下面所示，说明您成功进入了 Apollo 的运行容器：
   
   ```bash
   [user@in-runtime-docker:/apollo]$
   ```

##### b. 启动并打开 Dreamview+

您可以通过包管理或源码方式启动 Dreamview+，请您选择一种方式按照相应命令启动 Dreamview+。

- 方式一：包管理方式
  
  通过包管理方式进入 docker 环境中，在 docker 环境中执行以下命令启动 Dreamview+：
  
  ```bash
  aem bootstrap start --plus
  ```
  
  > 注意：
  > 
  > - 如果您想要停止 Dreamview+，请输入 `aem bootstrap stop --plus` ，
  > - 如果您想要重启 Dreamview+，请输入 `aem bootstrap restart --plus` 。

- 方式二：源码方式
  
  通过源码方式进入 docker 环境，在 docker 环境中执行以下命令启动 Dreamview+：
  
  ```bash
  bash scripts/bootstrap.sh start_plus
  ```
  
  > 注意：
  > 
  > - 如果您想要停止 Dreamview+，请输入 `bash scripts/bootstrap.sh stop_plus` ，
  > - 如果您想要重启 Dreamview+，请输入 `bash scripts/bootstrap.sh restart_plus` 。

启动成功后，在浏览器输⼊ `localhost:8888` ⽹址打开 Dreamview+ 界面。

#### 4. 启动 Camera 模块。

#### 5. 验证 Camera 是否工作正常。

如果一切正常，则会在 `DreamView` 右下角出现摄像头采集的图像。

在 `docker` 环境内使用 `cyber_monitor` 工具，查看 `/apollo/sensor/camera/front_6mm/image` 、 `/apollo/sensor/camera/front_12mm/image` 是否有数据输出，且帧率是否稳定在 15 帧左右。

```bash
//输入如下命令启动cyber_monitor
budaoshi@in_dev_docker:/apollo$ cyber_monitor
```

## NEXT

现在，您已经完成摄像头感知设备集成，接下来可以开始 [基于摄像头的封闭园区自动驾驶搭建--感知设备标定](../应用实践/车辆集成教程/基于摄像头的封闭园区自动驾驶搭建/感知设备标定.md)。

## 常见问题

### 规则文件修改后不显示软连接

规则文件修改后需要重启计算单元 (HW 3.0) 才能生效。
