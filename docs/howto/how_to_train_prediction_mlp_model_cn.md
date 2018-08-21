## How to train the MLP Deep Learning Model
### 前提条件

训练MLP深度学习模式有2个前提条件：

#### 下载并安装Anaconda

* 请从官网下载并安装Anaconda软件

#### 安装依赖项：

运行以下命令安装必要的依赖项：
*	**安装numpy**: conda install numpy
* **安装tensorflow**: conda install tensorflow
* **安装keras (version 1.2.2)**: conda install -c conda-forge keras=1.2.2
* **安装h5py**: conda install h5py
* **安装protobuf**: conda install -c conda-forge protobuf  

### 训练模型：
接下来要遵循的步骤是使用发布的演示数据来训练MLP模型。为了方便起见，我们把阿波罗作为本地阿波罗储存库的路径，例如，`/home/username/apollo`

1.	如果用来存储离线预测数据的文件夹不存在的话，则使用命令`mkdir APOLLO/data/prediction`创建一个新文件夹。

2.  打开`apollo/modules/prediction/conf/prediction.conf.`通过改变 `--noprediction_offline_mode` 成 `--prediction_offline_mode`来启动离线模式

3.  在apollo文件夹中用`bash docker/scripts/dev_start.sh` 启动dev docker

4.  在apollo文件夹中用`bash docker/scripts/dev_into.sh`进入dev docker

5.  在docker中，`/apollo/`下运行`bash apollo.sh build`进行编译

6.  在docker中，`/apollo/`下通过`python docs/demo_guide/rosbag_helper.py demo_2.0.bag`下载演示用rosbag

7.  在docker中，`/apollo/`下通过`bash scripts/prediction.sh start_fe` 运行预测模块

8.  打开一个新的终端窗口，使用步骤4进入apollo dev docker

9.  在新的终端窗口中，在`/apollo/`下，使用`rosbag play demo_2.0.bag`播放演示rosbag。
10. 当rosbag在新终端中运行结束后，在原来的终端按下`Ctrl + C`停止预测模块。
11. 在 `/apollo/data/prediction/`文件夹中检查是否有`feature.0.bin`文件存在

12. 在docker中进入`/apollo/modules/tools/prediction/mlp_train/`， 使用`python generate_labels.py -f /apollo/data/prediction/feature.0.bin`标记数据。然后在`/apollo/data/prediction/`文件夹中检查是否有`feature.0.label.bin`文件存在

> 更新: 在docker中进入`/apollo/modules/tools/prediction/mlp_train/`， 使用`python generate_labels.py /apollo/data/prediction/feature.0.bin /apollo/data/prediction/feature.0.label.bin` 标记数据。然后在`/apollo/data/prediction/`文件夹中检查是否有`feature.0.label.bin`文件存在

13. 在docker中，`/apollo/modules/tools/prediction/mlp_train/`下， 通过
`python generate_h5.py -f /apollo/data/prediction/feature.0.label.bin`生成 H5文件。然后检查`feature.0.label.h5`是否被创建

> 更新：在docker中，`/apollo/modules/tools/prediction/mlp_train/`下， 通过
 `python generate_h5.py /apollo/data/prediction/feature.0.label.bin  /apollo/data/prediction/feature.0.label.h5` 生成 H5文件。然后检查`feature.0.label.h5`是否被创建

14. 退出dev docker

15. 进入`APOLLO/modules/tools/prediction/mlp_train/proto/`文件夹并运行`protoc --python_out=./ fnn_model.proto`来生成fnn_model_pb2.py

16. 进入`APOLLO/modules/tools/prediction/mlp_train/`文件夹通过`python mlp_train.py APOLLO/data/prediction/feature.0.label.h5`启动训练模型

17. 模型的评估报告在`APOLLO/modules/tools/prediction/mlp_train/evaluation_report.log`

18. 模型会被存放在`APOLLO/modules/tools/prediction/mlp_train/mlp_model.bin`，如果你觉得你的模型更好可替换在`APOLLO/modules/prediction/data/mlp_vehicle_model.bin`中原来的模型
