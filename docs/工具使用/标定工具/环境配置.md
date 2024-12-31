
产品试用，首先联系 apollo 团队，为自己的 apollo 账号添加标定工具权限，打开 Dreamview+ 即可使用。

产品使用，请联系 apollo 团队签约购买，在官网申请开通并审批通过后，进行设备添加与产品线绑定后，在车端完成鉴权即可在 Dreamview+ 使用。

## 步骤一：环境部署

标定工具支持在 x86_64 和 aarch64 两种架构上运行，部署方式：：

```bash
# 下载apollo开发工程
git clone https://github.com/ApolloAuto/application-core
# 进入工程目录
cd application-core/
# 拉取最新的配置
git pull
# 环境配置：会识别主机系统是x86_64还是aarch64修改对应的.env 和 .workspace.json配置
bash setup.sh
# 启动容器
aem start_gpu
# 进入容器
aem enter
# 安装
buildtool build

# application-core里面profiles目录提供了基于1个16线lidar和2个camera的样例配置，您可以复制sample目录根据自己的车型修改里面的配置
# 选择您的profile，以使用sample profile为例：
aem profile use sample
# 注意：profile use命令可能会有一些warning提示，通常可以忽略。
```

## 步骤二：配置文件

### 配置文件说明

路径：`/apollo/modules/dreamview_plus_plugins/calibration_tool/conf/calibration_tool.pb.txt`

```bash
lidar_calib { # lidar标定配置
    collect_strategy { # 标定数据采集策略
        angle: 0.2 # 定位信息和前一帧角度差超过这个阈值作为一个有效帧记录，取值：度数/180°*π
        distance:2.0 # 定位信息和前一帧距离差超过这个阈值作为一个有效帧记录， angle和distance是或的关系，满足一个就记录
        total: 200 # 采集的帧数
    }
    use_odometry: true # pose信息源，true（默认）：/apollo/sensor/gnss/odometry，false：/apollo/localization/pose
    calibration_data_path: "/apollo/data/calibration/lidar_calibration"  # 每次标定创建{calibration_data_path}/{task_id}目录，里面包括采集数据(collection_data)和标定结果(result)目录
    imu_frame_id: "imu"
    lidar_list [ # lidar列表
        {
            name: "lidar16_back" # lidar名字，前端展示用
            lidar_frame_id: "lidar16_back"
            lidar_channel: "/apollo/sensor/lidar16/back/PointCloud2" # lidar驱动输出的channel名字
            extrinsics { # 初始外参的旋转矩阵和平移矩阵
                qw: 0.707
                qx: 0.0
                qy: 0.0
                qz: 0.707
                tx: 0.0
                ty: 0.0
                tz: 1.49
            }
            output_filename: "/apollo/modules/perception/data/params/lidar16_back_extrinsics.yaml" # 标定结果文件路径(文件要存在并有写入权限)
        }
    ]
    calib_height: false # 是否标高度，true是，false否
}

camera_calib { # camera标定配置
    calibration_data_path: "/apollo/data/calibration/camera_calibration"  ## 每次标定创建{calibration_data_path}/{task_id}目录，里面包括采集数据(collection_data)和标定结果(result)目录
    camera_list [ # camera列表
        {
            name: "camera_front_12mm" # camera名字，前端展示用
            camera_channel: "/apollo/sensor/camera/front_12mm/image" # camera动输出的channel名字
            camera_frame_id: "camera_front_12mm"
            lidar_channel: "/apollo/sensor/lidar16/back/PointCloud2" # lidar驱动输出的channel名字
            lidar_frame_id: "lidar16_back"
            lidar_rotation: 0 # lidar点云旋转角度，默认是0度，逆时针为正
            intrinsics_filename: "/apollo/modules/perception/data/params/camera_front_12mm_intrinsics.yaml" # 内参文件路径
            output_filename: "/apollo/modules/perception/data/params/camera_front_12mm_extrinsics.yaml" # 标定结果文件路径(文件要存在并有写入权限)
            lidar_to_imu_extrinsic_file_path: "/apollo/modules/perception/data/params/lidar16_back_imu_extrinsics.yaml" #lidar-imu外参路径
        },
        {
            name: "camera_front_6mm" # camera名字，前端展示用
            camera_channel: "/apollo/sensor/camera/front_6mm/image" # camera动输出的channel名字
            camera_frame_id: "camera_front_6mm"
            lidar_channel: "/apollo/sensor/lidar16/back/PointCloud2" # lidar驱动输出的channel名字
            lidar_frame_id: "lidar16_back"
            lidar_rotation: 0 # lidar点云旋转角度，默认是0度，逆时针为正
            intrinsics_filename: "/apollo/modules/perception/data/params/camera_front_6mm_intrinsics.yaml" # 内参文件路径
            output_filename: "/apollo/modules/perception/data/params/camera_front_6mm_extrinsics.yaml" # 标定结果文件路径(文件要存在并有写入权限)
            lidar_to_imu_extrinsic_file_path: "/apollo/modules/perception/data/params/lidar16_back_imu_extrinsics.yaml" #lidar-imu外参路径
        }
    ]
}
```

`pre_rotation angle` 设置：在 Apollo 框架下，这个值设置如下图所示，这个值的含义是将相机顺时针旋转这个角度，其朝向和激光雷达的朝向能保持一致：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_1c7b350.png)


## 步骤三：启动服务

1. 在 docker 环境下输入以下命令启动 Dreamview+：

   ```bash
   aem bootstrap restart --plus
   ```

   在浏览器端通过访问机器的8888端口打开 Dreamview+ 页面。

2. 页面左下角有登录信息说明登录成功。

   ![3.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/3_5341e20.png)

3. 模式选择中选择 **Lidar Calibration** 开始 lidar 标定。参见 [lidar 标定](./激光雷达标定.md) 相关文档。
4. 模式选择中选择 **Camera Calibration** 开始相机标定。参见 [camera 标定](./相机标定.md) 相关文档。

## 步骤四：结果目录说明

结果都在工程的`data/calibration`目录下保存。

### lidar 标定结果

一般在`data/calibration/lidar_calibration`目录，以时间戳命名记录每次 lidar 标定数据，比如lidar2imu_171232645，其中：
* collection_data目录保存采集的点云和位置信息，
* result目录保存标定结果：找到对应 lidar 的目录，result.pcd 是标定融合的 pcd，result.yaml 中记录标定外参结果值。
### camera 标定结果

一般在`data/calibration/camera_calibration`目录，以时间命名每次 camera 标定数据，比如camera2lidar_20240405220953，其中：
* 以配置的 camera 名字开头命名的保存原始的 camera 图像和 lidar 点云对，result 目录保存每次采集对应的结果，
* failed_data 目录存放采集失败的图像点云对，
* extrinsic.yaml 存放最终的外参信息。

