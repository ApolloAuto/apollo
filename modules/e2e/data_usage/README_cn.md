# 快速入门
*	**<font size=4>data_usage目录提供的是demo示例，帮助开发者快速使用百度地图采集车采集的开源数据。</font>**
*	**<font size=4>模型需要迁移到在线系统上，迁移后的模型暂未开放。</font>**
## 训练模型
	sh run_train.sh
训练集下载到当前工程./trainsets目录下。
## 模型预测
	sh run_predict.sh  path/to/steering_model  path/to/acc_model  path/to/test_img  path/to/result

## 关于
* 数据集下载：请参考 [数据集下载](https://console.bce.baidu.com/apollo/task/download#/apollo/home)
* 数据集概览：请参考 [data_overview.md](https://github.com/ApolloAuto/apollo/blob/master/modules/e2e/model/data_overview.md)
* 模型概览：请参考 [model_overview.md](https://github.com/ApolloAuto/apollo/blob/master/modules/e2e/model/model_overview.md)