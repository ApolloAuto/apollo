# 参考线平滑算法

_**Tip**: 为了更好的展示本文档中的等式，我们建议使用者使用带有[插件](https://chrome.google.com/webstore/detail/tex-all-the-things/cbimabofgmfdkicghcadidpemeenbffn)的Chrome浏览器，或者将Latex等式拷贝到[在线编辑公式网站](http://www.hostmath.com/)进行浏览。_

二次规划（QP）+样条插值

## 1. 目标函数

### 1.1 分段寻路路径

将寻路路径划分为 **n** 段，每段用2个多项式表示：

```
$$
x = f_i(t)
  = a_{i0} + a_{i1} * t + a_{i2} * t^2 + a_{i3} * t^3 + a_{i4} * t^4 + a_{i5} * t^5
$$
```

```
$$
y = g_i(t) = b_{i0} + b_{i1} * t + b_{i2} * t^2 + b_{i3} * t^3 + b_{i4} * t^4 + b_{i5} * t^5
$$
```

### 1.2 定义样条段优化目标函数

```
$$
cost =
\sum_{i=1}^{n}
\Big(
\int\limits_{0}^{t_i} (f_i''')^2(t) dt
+ \int\limits_{0}^{t_i} (g_i''')^2(t) dt
\Big)
$$
```

### 1.3 将开销（cost）函数转换为QP公式

QP公式：

```
$$
\frac{1}{2} \cdot x^T \cdot H \cdot x + f^T \cdot x
\\
s.t. LB \leq x \leq UB
\\
A_{eq}x = b_{eq}
\\
Ax \leq b
$$
```

## 2 约束条件

### 2.1 平滑节点约束

该约束的目的是使样条的节点更加平滑。假设两个段$seg_k$ 和$seg_{k+1}$互相连接，且$seg_k$的累计值 **s** 为$s_k$。计算约束的等式为：

```
$$
f_k(s_k) = f_{k+1} (s_0)
$$
```

同样地，该公式也适用于下述等式：

```
$$
f'_k(s_k) = f'_{k+1} (s_0)
\\
f''_k(s_k) = f''_{k+1} (s_0)
\\
f'''_k(s_k) = f'''_{k+1} (s_0)
\\
g_k(s_k) = g_{k+1} (s_0)
\\
g'_k(s_k) = g'_{k+1} (s_0)
\\
g''_k(s_k) = g''_{k+1} (s_0)
\\
g'''_k(s_k) = g'''_{k+1} (s_0)
$$
```

### 2.2 点采样边界约束

在路径上均匀的取样 **m** 个点并检查这些点的预定义边界。

```
$$
f_i(t_l) - x_l< boundary
\\
g_i(t_l) - y_l< boundary
$$
```
