## 步骤一：启动 Dreamview+

您可以通过包管理或源码方式启动 Dreamview+，请您选择一种方式按照相应命令启动 Dreamview+。

### 1. 启动 Dreamview+

#### 方式一：包管理方式

通过包管理方式进入 docker 环境中，在 docker 环境中执行以下命令启动 Dreamview+：

```bash
aem bootstrap start --plus
```

> 注意：
> 
> - 如果您想要停止 Dreamview+，请输入`aem bootstrap stop --plus`，
> - 如果您想要重启 Dreamview+，请输入`aem bootstrap restart --plus`。

#### 方式二：源码方式

通过源码方式进入 docker 环境，在 docker 环境中执行以下命令启动 Dreamview+：

```bash
bash scripts/bootstrap.sh start_plus
```

> 注意：
> 
> - 如果您想要停止 Dreamview+，请输入`bash scripts/bootstrap.sh stop_plus`，
> - 如果您想要重启 Dreamview+，请输入`bash scripts/bootstrap.sh restart_plus`。

### 2. 打开 Dreamview+

启动成功后，在浏览器输⼊ `localhost:8888` ⽹址打开 Dreamview+ 界面。

当出现如下界面，表示 Dreamview+ 启动成功了。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_8455c10.png)

点击左下角 **个人中心** > **设置** > **全局设置** ，可以选择界面语言类型。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_ce0ce76.png)

## 步骤二：使用 SimControl 仿真自动驾驶场景

1. 在 **模式** 设置中选择 **PNC 模式** ，并启动 **Planning** 模块，选择 **Sim_Control** 操作模式，高精地图选择 **Sunnyvale Big Loop** ，车辆选择 **MKZ Example** 。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_fa3ada7.png)

2. 点击 **车辆可视化** 面板的 **路由编辑** 功能，进入车辆路由设置界面。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_669f410.png)

3. 分别设置起点和轨迹点（最后一个轨迹点为终点），设置完成后点击保存编辑。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4ac1090.png)

4. 回到主界面后，点击左下角启动按钮，即可看到车辆开始在仿真环境中运行。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_4d87528.png)
