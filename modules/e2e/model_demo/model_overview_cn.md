# 模型说明文档
## 横向模型

横向模型即计算曲率模型。

`代码文件：run_model.py`
### 生成数据

```
def generator_data_steering():
```

异常值过滤：

```
def filter(c,v):
	return v > 1 and c < 0.5

```
顺序生成数据，因采集的数据含有大量直行（曲率小于等于0.0001）的数据。过滤异常值后，选取10%的直行数据。

```
if filter(curv, speed) and (curv > 0.0001 or random.randint(1, 10) == 1)
```
### CNN模型
`模型文件：src\steering_model.py`

<img src="./png_resource/steering_model1.png" width = "300" height = "320" align=center />
<img src="./png_resource/steering_model2.png" width = "300" height = "320" align=center />
<img src="./png_resource/steering_model3.png" width = "300" height = "320" align=center />



## 纵向模型
纵向模型即计算加速度。

`代码文件：run_model.py`
### 生成数据

```
def generator_data_acc():
```

异常值过滤：

```
def filter(c,v):
	return v > 1 and c < 0.5

```
顺序读取数据，定义time_step＝5，过滤异常数据，选取顺序的五帧图，求得的第五帧图时刻的加速度值作为输入。
每帧图间隔0.125s，第五帧的加速度

<p>
$$ ACC_5 = \frac{(V_6-V_4)}\{0.25} $$
</p>

如果第六帧速度值经filter被过滤掉，则丢弃该样本，即这五帧图。

```
next_curv = attrs[i+1][4]
next_speed = math.sqrt(attrs[i+1][1] ** 2 + attrs[i+1][2] ** 2)
if not filter(next_curv, next_speed):
	step_cnt = 0
	continue
else:
	acc = (next_speed - step_v[3]) / t_sep

```

### Conv_LSTM模型
`模型文件:src\acc_model.py`

<img src="./png_resource/acc_model.png" width = "300" height = "320" align=center />
