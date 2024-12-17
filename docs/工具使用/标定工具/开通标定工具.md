本小节以开通申请为例说明申请标定工具，您也可以提交免费试用申请并等待审批通过后，即可使用。

> 说明：开通申请是针对已购买标定工具的客户，根据购买合同号与服务内容提交开通申请。目前面向机构账户开放。

## 前提条件

- 完成鉴权工具的安装。请您在 docker 环境外执行鉴权工具安装命令，详情参见文档 [Apollo 软件安装](https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZAWnAQCcBrZgcwFMAdn3Z5SsVAhB9IIWBQIB9GnQUAmBQA8AogDZNATgDMmgBwARTVoAsmgEIWN+gMKW9jtyYDsrzQEEAjH7G6tomdg6+AGKWNhq2tnZuWvqmxhq+YQBmAAwmhqQEngCs+kb62dhF-pl8pNl8mapWfPVF9fWYoBSoQA) > **环境工具安装** > **鉴权工具安装** 小节。

- 请确保 lidar、camera 硬件驱动完成安装，以便打开 Dreamview+ 进行标定时，camera、lidar 可正常启动。

## 步骤一：申请开通服务

1. 登录 [Apollo 开发者社区](https://apollo.baidu.com/)。

2. 在菜单栏选择 **产品** > **开发工具** > **地图与标定** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_00474f7.png)

3. 选择工作台身份，并单击 **确认** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_6419eb4.png)

4. 在 **地图与标定** 申请页面，单击 **开通申请** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_353bee1.png)

   > 说明：标定功能需要申请，无论您选择免费试用或者选择开通申请，需等待相关人员审批通过之后才可使用。

5. 在 **开通申请** 页面中，编辑申请的工具类型等相关信息并单击 **确定** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_0e68cdc.png)

   显示申请标定工具成功。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_61144e1.png)

   申请之后，显示您申请的服务在审批中，请您耐心等待审批。待审批通过后，返回配置管理完成下面的工具激活操作。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_5e45d98.png)

## 步骤二：激活服务

1. 在 **配置管理** 中，选择 **标定工具** 页签。
2. 单击 **激活** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_a5a5ee7.png)

3. 单击 **确认** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_73e9fb0.png)

   显示标定工具激活成功。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_bbb0937.png)

   在 **服务权益** 中可以查看到申请的标定工具已经开通。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_70a0b84.png)

## 步骤三：设备管理

### 1. 添加设备

1. 单击 **设备管理** > **添加设备** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_b416c9e.png)

2. 在弹出的 **添加设备** 窗口中，编辑设备信息。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_fad3dd1.png)

3. 单击 **提交** 。

### 2. 绑定产品线

1. 单击 **绑定产品线** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_ab3c626.png)

2. 勾选产品线，并单击 **确定** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_a16f61d.png)

   > 说明：绑定产品线之后不可修改，请您确保提交信息正确。

   显示产品线绑定成功。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_924dd73.png)

## 步骤四：登录 Dreamview+

### 1. 个人账户登录

1. 登录个人账户 [Apollo Studio](https://apollo.baidu.com/workspace/account-center/services)。

2. 选择 **车端登录** 页签，点击 **生成** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_a83b979.png)

   > 说明：如果点击过生成按钮，则页面只显示一键复制。

3. 进入工控机软件 apollo 版本目录位置，输入以下命令进入 docker 环境：

   ```bash
   cd apollo 版本目录位置
   aem enter
   ```

4. 点击 **一键复制** ，然后在 docker 环境内复制插件信息，按回车键。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_89bae24.png)

### 2. 启动服务

1. 在 docker 环境下输入以下命令启动 Dreamview+：

   ```bash
   aem bootstrap restart --plus
   ```

   在浏览器端通过访问机器的8888端口打开 Dreamview+ 页面。

2. 页面左下角有登录信息说明登录成功。

   ![3.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/3_5341e20.png)

## 步骤五：车端注册激活

### 1. 绑定车辆和设备

1. 在 **Settings** > **Account** 中，选择创建的机构，并单击 **close** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_ba39b4c.png)

2. 在 **Device** 中，勾选要与物理车辆绑定的云端逻辑设备。
3. 单击 **Device registration** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_8a0f0ec.png)

### 2. 车辆注册

显示标定工具在审批中，请您联系平台内部人员进行鉴权审批。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_1720132.png)

### 3. 车辆激活

证书完成审批后，会显示待激活状态。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_441d08a.png)

单击 **Activation** 激活，显示为已激活状态。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_253ee09.png)

激活完成后，你即可在车端 Dreamview+ 中使用标定服务（Camera 标定与 Lidar 标定）。

> 说明：注册和激活完成后，云端会实时同步显示车端的注册和激活状态，并在云端的设备列表里进行展示。
