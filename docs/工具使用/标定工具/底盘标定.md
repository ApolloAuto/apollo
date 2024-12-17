## 概述

是底盘线控功能性能评测工具。 用于验证 Apollo 下发的控制指令，车辆底盘能否准确并快速执行。

### 功能布局说明
![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_4b866da.png)

## 前提条件

- 完成鉴权工具的安装
- 开通标定服务且完成鉴权

## 环境配置

### 步骤一：环境部署

参见标定工具[环境配置](./环境配置.md)。

### 步骤二：配置文件
#### 参数说明

以下为配置文件`teleop_conf.pb.txt`，在`/apollo/modules/dreamview_plus_plugins/teleop_tool/conf/`路径下，建议按此默认设置即可。

```bash
action_conf {
  #单次命令检测最长等待时间
  maximum_waiting_time: 3.0
}

gear_conf {
  # 车辆档位配置 D档位， R档位， N档位， P档位
  gear: GEAR_DRIVE
  gear: GEAR_REVERSE
  gear: GEAR_NEUTRAL
  # P档位可选配
  gear: GEAR_PARKING
  record_curve: false
  maximum_waiting_time: 3.0
}

steer_wheel_conf {
  #自动检测默认步长
  step: 5.0
  #是否记录底盘响应曲线和命令下发曲线
  record_curve: true
  #底盘延时时间阈值
  delay_time_threshold: 0.2
  #单次命令检测最长等待时间
  maximum_waiting_time: 3.0
  #峰值最大角度差值
  peak_maximum_angle_difference: 2 
  #稳态角度最大差值
  maximum_angle_difference: 2
  #稳态幅度值
  steady_state_amplitude: 0.5
  #稳态帧数
  steady_state_frame_number: 20
}

brake_conf {
  #自动检测默认步长
  step: 5.0
  #是否记录底盘响应曲线和命令下发曲线
  record_curve: true
  #底盘延时时间阈值
  delay_time_threshold: 0.2
  #单次命令检测最长等待时间
  maximum_waiting_time: 3.0
  #稳态幅度值
  steady_state_amplitude: 0.5
  #稳态帧数
  steady_state_frame_number: 20
  #稳态百分比最大差值
  maximum_percentage_difference: 2
}

parking_brake_conf {
  #是否记录底盘响应曲线和命令下发曲线
  record_curve: false
  #单次命令检测最长等待时间
  maximum_waiting_time: 3 
}

throttle_conf {
  #自动检测默认步长,一般不超过10%
  step: 5.0
  #是否记录底盘响应曲线和命令下发曲线
  record_curve: true
  #底盘延时时间阈值
  delay_time_threshold: 0.3
  #单次命令检测最长等待时间
  maximum_waiting_time: 3.0
  #稳态幅度值
  steady_state_amplitude: 0.5
  #稳态帧数
  steady_state_frame_number: 20
  #稳态百分比最大差值
  maximum_percentage_difference: 2
  #油门检测可达最大百分比
  maximum_test_percentage: 10
}

speed_conf {
  record_curve: true
  #陡峭值，速度检测突变可接受最大范围值
  jagged_threshold: 0.3
  delay_time_threshold: 0.02
}
#can总线刷新帧率
canbus_clock: 10.0

```

#### 修改参数

更改 canbus 的配置参数， 使 canbus 能进入调试， 文件路径`/apollo/modules/canbus/conf/canbus.conf`。


```bash
--enable_chassis_detail_pub
 --enable_chassis_detail_sender_pub
 --chassis_debug_mode=true
 --use_control_cmd_check=false
 --use_guardian_cmd_check=false
 --noreceive_guardian
 --estop_brake=30.0
 ```
 
 
### 步骤三：启动服务 

1. 在 docker 环境下输入以下命令启动 Dreamview+：

   ```bash
   aem bootstrap restart --plus
   ```
   
   在浏览器端通过访问机器的8888端口打开 Dreamview+ 页面。

   页面左下角有登录信息说明登录成功。

   ![3.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/3_5341e20.png)
   
4. 在 **模式** 中选择 **Canbus Debug**。
5. 在 **模块** 中，启动 **Canbus** 模块。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_3db2433.png)

6. 将遥控器切换到自动驾驶模式。
 
## 操作步骤


### 自动握手检测


检测进退自动以及接管的指令下发后结果正确性及延迟时间。需要手动检测。

#### 1. 准备工作


选择 **Canbus Debug** 模式，启动 **Canbus** 模块后会自动进入自动握手检测界面。

1. 关闭车辆上的自动驾驶开关，单击 **开始检测**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_648e95f.png)

2. 打开车辆上的自动驾驶开关，然后进行下一步操作。


#### 2. 进自动驾驶

1. 单击 **进自动驾驶** 的 **检测** 按钮。

2. 根据提示，确认车辆上的自动驾驶开关是否已打开，如已打开，单击 **已打开**；如未打开，需先打开然后才能打开 TeleopTool 的自动驾驶。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_4a8304a.png)
   
   结果显示检测 **正确**。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_6c88745.png)

#### 3. 退自动驾驶

1. 单击 **退自动驾驶** 的 **检测** 按钮。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_dd3781a.png)
   
   结果显示检测 **正确**。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_26b4764.png)
   

#### 4. 进自动驾驶

1. 单击 **进自动驾驶** 的 **检测** 按钮。
2. 根据提示，先打开车辆上的自动驾驶开关，才可以打开 TeleopTool 的自动驾驶，确认并单击 **已打开**。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_bf93664.png)
   
   结果显示检测 **正确**。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_f6f89dc.png)
   

#### 5. 接管测试
 
1. 单击 **接管测试** 的 **检测** 按钮。
2. 手动控制遥控器，切换驾驶模式为手动。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_cae5aa4.png)
   
   结果显示检测 **正确**。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_e20f3b8.png)
    
    
#### 6. 进自动驾驶

1. 单击 **进自动驾驶** 的 **检测** 按钮。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_dc899f0.png)
 
   结果显示检测 **正确**。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_cfe5fa7.png)
   
2. 检测完成后，当检测结果全部显示 **正确**，单击 **下一步** 进入档位测试或选择任意项进行测试。
   
   >注意：如果检测结果显示 **错误**，可以单击 **详情** 排查原因后重新检测。
   
  

### 档位检测

档位检测：检测D、R、N、P档位指令下发后结果正确性以及延迟时间。支持自动检测。默认：自动检测。

#### 1. 准备工作

1. 进入档位测试后，将车轮驱动轮悬空，并单击 **我已完成** 按钮。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_e40c5a4.png)

2. 打开车辆的自动驾驶开关，并单击 **我已完成** 按钮。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_f5cc20c.png)

3. 打开自动驾驶模块开关，并单击 **我已完成** 按钮。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_7c4b9c8.png)

4. 单击 **开始检测**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_b2e13d9.png)
   
   >说明：进行自动检测前，需要先释放车端的手刹按钮。


#### 2. 自动检测

1. 单击 **开始** 进行自动检测。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_b49b32f.png)
   
   当下发指令和返回结果显示的档位信息一样，则显示检测结果正确，否则，显示检测结果错误。
   
   >说明：
   >- 档位检测为纯自动检测，需要检测的档位要在配置文件里配置好，默认 D R N P 档。如没有 P 档，可从`teleop_conf.pb.txt`中`gear_conf`配置中去掉 P 档配置。
   >- 自动检测完成后，手动检测档位时，如果出现异常，需再次手动检测即可显示正常。
   
2. 检测完成后，当检测结果全部显示 **正确**，单击 **下一步** 进入方向盘测试或选择任意项进行测试。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_53b4f85.png)
   

### 方向盘检测

左右转向期间，检测下发指令与返回结果一致性以及延迟时间。支持手动检测、阶越检测和连续检测。默认：手动检测。

#### 1. 准备工作

1. 进入方向盘检测后，将车辆驱动轮悬空，并单击 **我已完成**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_1d9b8bb.png)
   
2. 将方向盘分别向左、向右转到最大后，点击 **查看详情** 查看方向盘详情中的steering_angle数值。
 
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_a22c5cd.png)
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_de33e2a.png)
   
3. 将方向盘详情中的steering_angle数值依次输入左转最大真值和右转最大真值输入框中，并单击 **我已完成**。 
4. 根据提示调整左右最大真值并单击 **确认**。

   左右最大真值自动调整完成并进入下一步。
   
5. 设置转速，并单击 **我已完成**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_2b0534f.png)
   
    >说明：
    >- 如果无法知道底盘的转速，这项可不填入，单位：度/秒。
    >- 在该项 **我已完成** 后，canbus 模块会自动杀死重启，请查看 canbus 是否重启成功。
   
6. 打开车辆的自动驾驶开关，并单击 **我已完成**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_1a0d2c7.png)
   
7. 单击 **我已完成** 进入自动驾驶模式。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_6c23ca5.png)
   
   >说明：如果切换自动驾驶模式时车辆自动拉起手刹，需要手动释放手刹按钮。请根据车辆实际情况释放手刹按钮。

8. 以上步骤均完成后，单击 **开始检测**。


  
#### 2. 添加指令并检测

您可以通过手动添加指令检测、阶跃检测或者连续检测的方式进行。

##### 手动添加指令检测

1. 设置旋转值，并单击 **添加左转** 进行左转检测。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_a9e8b09.png)
     
2. 设置旋转值，并单击 **添加右转** 进行右转检测。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_09ae13d.png)
     
   >说明：添加第一条命令成功后，再次执行第二条命令，数值变化是根据上一条命令累积执行的。以此类推。
     
3. 设置旋转到，并单击 **添加**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_6c21210.png)

   >说明：
   >- 旋转值指左右转方向盘角度步长值，相对值，通过加减逐步叠加。
   >- 旋转到指绝对值。
   >- 旋转到的值不能直接从一个方向打满切换到另外一个方向打满。
   
4. 单击 **曲线图** 查看响应时间曲线。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_3c3c80d.png)
   
   如果下发指令和返回结果曲线重合，且延时不超过300ms（支持在配置文件中配置）表示合格。
   
   >说明：在曲线图中，您可以点击 **下发指令**，关闭下发指令曲线，只显示返回结果曲线。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_76b5d0c.png)
   
   或者关闭返回结果曲线，只显示下发指令曲线。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_62c8daf.png)
   
   
   
##### 阶跃检测 

1. 单击 **阶跃检测** 进入阶跃检测界面。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_2037a86.png)
     
2. 单击 **重新设置**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_b3fd202.png)

    >说明：系统默认会给出5%的步长值，如果检测其他数值，可以点击重新设置按钮进行设置。
     
 3. 在弹出的步长设置界面设置 **步长值**，并单击 **确认**。

    ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_f422982.png)
     
     >说明：步长值需要大于`teleop_conf.pb.txt`中的稳态角度最大差值。
     
4. 单击 **开始** 进入检测。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_e8d6d15.png)
    
5. 单击 **曲线图** 查看响应时间曲线。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_e7c0874.png)

   如果下发指令和返回结果曲线重合，说明检测合格，如不重合，则需要重新检测。


##### 连续检测

1. 单击 **连续检测**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_c67c5ab.png)
     
2. 设置幅值和周期，并单击 **添加** 开始检测。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_e8815c2.png)
     
   >说明：
   >- 连续检测：指 control 命令以 canbus 总线刷新帧率进行定频发送，检测底盘跟随响应情况。
   >- 幅值：连续检测中需要方向盘打到的左右最大百分比。
   >- 周期：所有命令下发完一轮的时间，这里周期最好大于16s。
     
   检测完成后查看曲线图，如果两个曲线的峰值差距不大，且两个曲线在同一幅度时，延迟时间不大，则说明检测合格。

   检测完成后，单击 **下一步** 进入刹车测试或选择任意项进行测试。


### 刹车检测

检测刹车踏板指令下发后结果正确性以及延迟时间。支持手动检测、自动检测。默认：手动检测。

#### 1. 准备工作

1. 将车辆驱动轮悬空，并单击 **我已完成**。
  
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_ad56698.png)

2. 打开车辆上的自动驾驶开关，并单击 **我已完成**。
 
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_48aab70.png)

3. 打开自动驾驶模块，并单击 **我已完成**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_353c024.png)
   
4. 依次完成以上步骤后，单击 **开始检测**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_e4269ff.png)
   
   >说明：如果车辆带手刹功能，需要先释放手刹按钮。

#### 2. 添加指令进行检测

##### 手动添加指令检测

1. 设置踏板量，单击 **添加踩下**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_edaa7a8.png)
   
   >说明：稳态百分比最大差值不超过2%。
   
2. 设置踏板量，单击 **添加松开**。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_390c597.png)
   
   查看曲线图。如果下发指令和返回结果曲线重合，或者稳态百分比最大差值不超过±2%，都说明检测合格，如不重合且大于±2%，则需要重新检测或调整底盘参数。 
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_2028425.png)
   
3. 单击 **下一步** 进入手刹测试或选择任意项进行测试。

##### 自动检测


说明：开始自动检测前，需检查车辆手刹是否放下，如果拉起状态会显示检测错误。

1. 单击 **自动检测**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_4774196.png)
   
2. 单击 **重新设置**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_c4c0ba5.png)

   >说明：系统默认会给出5%的步长值，如果检测其他数值，可以点击重新设置按钮进行设置。

3. 在弹出的步长设置界面设置 **步长值**，并单击 **确认**。
   
4. 单击 **开始** 进入检测。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_7301543.png)
   
5. 单击 **曲线图** 查看响应时间曲线。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_70ee721.png)

   >说明：如果检测不合格，可以重测，如果多次重测还是不合格，说明底盘有问题，需要检查底盘。


### 手刹检测

检测手刹拉起、放下指令下发后结果正确性以及延迟时间。支持自动检测。默认：自动检测。

>说明：如果车型无手刹，可跳过该检测直接进入油门检测。

#### 1. 准备工作

1. 将车辆驱动轮悬空，并单击 **我已完成**。
  
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_ce0d79a.png)

2. 打开车辆上的自动驾驶开关，并单击 **我已完成**。
 
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_c197fbb.png)

3. 打开自动驾驶模块，并单击 **我已完成**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_691b016.png)
   
4. 依次完成以上步骤后，单击 **开始检测**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_2611817.png)
   
   >说明：开始检测前，放下手刹。
   
#### 2. 自动检测

1. 单击 **开始**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_44cfd82.png)

   系统开始自动检测。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_5b91bb2.png)

   检测显示结果正确的话，则检测通过。否则，需要查看车端手刹和检测项是否一致，如果不一致说明车辆底盘参数有问题，需要进行修改。
   
2. 单击 **下一步**，进行油门检测选择任意项进行测试。
   
   



### 油门检测

检测车辆在不同档位下（D、R、N），下发指令与返回结果一致性及延迟时间。支持手动检测、自动检测。默认：手动检测。

#### 1. 准备工作

1. 将车辆驱动轮悬空，并单击 **我已完成**。
  
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_f5f113d.png)

2. 打开车辆上的自动驾驶开关，并单击 **我已完成**。
 
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_b986a06.png)

3. 打开自动驾驶模块，并单击 **我已完成**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_db1228e.png)
   
4. 依次完成以上步骤后，单击 **开始检测**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_1dc849b.png)
   
   >说明：开始检测前，放下手刹。


#### 2. 设置检测目标

>说明：为了安全起见，油门有最大踏板量限制，在配置文件里，目前默认 10%，如需检测更大油门，需要修改油门配置。


##### 手动检测

1. 选择档位。
2. 设置踏板量，并单击 **添加踩下**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_4b49a45.png)
   
3. 单击 **添加松开**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_b25ef5f.png)

   查看曲线图，如果下发指令和返回结果曲线重合，说明检测合格，如不重合，则需要重新检测。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_5c347b2.png)
   
   >说明：松开踏板后，车可能会继续前进一段距离，如果车辆速度过高，可以按刹车键减速，长按刹车键刹车百分比会上升。
   
   
   
##### 自动检测



1. 单击 **重新设置**。

   >说明：自动检测模式默认是 5% 的步长值，您也可以重新设置步长值。
   
3. 输入 **步长值**，并单击 **确认**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_79102f9.png)
   
3. 单击 **开始**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_fc7da47.png)

   检测完成后，查看曲线图，如果返回指令曲线拟合到下发指令，且延迟时间小于配置中的限制，则检测通过。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_6d2f052.png)

4. 单击 **下一步**，进行速度检测选择任意项进行测试。
   
   




### 速度检测

检测手动驾驶模式下，切换不同档位，速度的实时变化曲线。支持手动检测。默认：手动检测。

#### 1. 准备工作

1. 关闭车辆上的自动驾驶开关，将车辆行驶到宽敞的道路中，如果切换模式时手刹自动拉起，需要先将手刹放下，并单击 **我已完成**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_ce3ea7c.png)

2. 挂 D 挡，确认车辆行驶方向 **前进**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_2e3a9a4.png)

3. 挂 R 挡，确认车辆行驶方向 **后退**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_33ab30c.png)

4. 挂 N 挡，确认车辆行驶方向 **驻车**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_2b629cd.png)
   
5. 单击 **开始检测**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_a92ea1c.png)
  

#### 2. 设置检测目标

1. 依次选择不同档位。
2. 设置最高速度，并单击 **确认**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_2ffd61a.png)
   
   驾驶车辆看车辆是否能达到设置的最高速度。
  
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_0a15f2f.png)

   >说明：只要曲线能达到设置的最高速度值，检测通过。



