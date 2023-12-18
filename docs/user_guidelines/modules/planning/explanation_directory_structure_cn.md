# 目录结构

在新版本中对 planning 进行了分层，主要分基础包和插件两层，基础包是 planning 开发的基础依赖层，插件是用户可以自定义和扩展的功能包集合。

![image (5).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%285%29_1dcf270.png)

下面对 planning 中的重要目录和功能进行介绍。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_0c73905.png)

`planning_base` 目录包含了 planning 所有的基础算法依赖和 planning 父类接口。

![image (1).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%281%29_747f770.png)

`scenarios` 目录包含了现有的场景插件，每一个子目录是一个独立的插件包。

![image (2).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%282%29_c26e9f3.png)

`tasks` 目录包含了所有的任务插件。

![image (3).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%283%29_46d5e9e.png)

`traffic_rules`目录包含了所有的 traffic rule 插件。

![image (4).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%284%29_d841a55.png)

`pnc_map`目录包含了 pnc_map 插件，pnc_map 插件是为了根据不同地图类型和外部命令，生成相应的参考线，供后续场景和任务处理使用。

![image (6).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%286%29_1c44e75.png)
