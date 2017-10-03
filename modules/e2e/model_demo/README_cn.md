# 快速入门
**<font color=##FF0000 size=3>model_demo目录提供的是模型的demo示例，意在让开发者快速了解开源数据格式及如何使用已开源的数据，我们也会不断开放end-to-end的模型和算法。</font>**
## 训练模型
	sh run_train.sh
训练集下载到当前工程./trainsets目录下。
## 模型预测
	sh run_predict.sh  path/to/steering_model  path/to/acc_model  path/to/test_img  path/to/result

## 关于
* 数据集下载：请参考 [数据集下载](https://console.bce.baidu.com/apollo/task/download#/apollo/home)
* 数据集概览：请参考 [data_overview.md](https://github.com/ApolloAuto/apollo/blob/master/modules/e2e/model/data_overview.md)
* 模型概览：请参考 [model_overview.md](https://github.com/ApolloAuto/apollo/blob/master/modules/e2e/model/model_overview.md)