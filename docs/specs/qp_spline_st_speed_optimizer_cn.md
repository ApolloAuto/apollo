# 二次规划ST速度优化

_**Tip**: 为了更好的展示本文档中的等式，我们建议使用者使用带有[插件](https://chrome.google.com/webstore/detail/tex-all-the-things/cbimabofgmfdkicghcadidpemeenbffn)的Chrome浏览器，或者将Latex等式拷贝到[在线编辑公式网站](http://www.hostmath.com/)进行浏览。_

## 1  定义

从二次规划样条路径中选取一条路径后，Apollo将路线上的所有障碍物和自动驾驶车辆（ADV）展现在一个时间-路径图上（path-time ST），该路径图表示了路径上的站点变化。速度优化的任务是在ST图上找到一条合理的，无障碍的路径。

Apollo使用多个样条来表示速度参数，在ST图上表示为一系列的ST点。Apollo会对二次规划的结果做再次的平衡以获得最佳的速度参数。QP问题的标准类型定义为：

```
$$
minimize \frac{1}{2} \cdot x^T \cdot H \cdot x + f^T \cdot x 
\\
s.t. LB \leq x \leq UB
\\
A_{eq}x = b_{eq}
\\
Ax \leq b
$$
```

## 2  目标函数

### 2.1  获取样条段

将路ST速度参数分为 **n** 段，每段路径用一个多项式来表示。

### 2.2  定义样条段函数

每个样条段 ***i*** 都有沿着参考线的累加距离$d_i$。每段的路径默认用5介多项式表示。多项式介数可以通过配置参数进行调整。

```
$$
s = f_i(t) 
  = a_{0i} + a_{1i} \cdot t + a_{2i} \cdot t^2 + a_{3i} \cdot t^3 + a_{4i} \cdot t^4 + a_{5i} \cdot t^5
$$
```

### 2.3  定义样条段优化函数

Apollo首先定义$cost_1$以使路径更加平滑：

```
$$
cost_1 = \sum_{i=1}^{n} \Big( w_1 \cdot \int\limits_{0}^{d_i} (f_i')^2(s) ds + w_2 \cdot \int\limits_{0}^{d_i} (f_i'')^2(s) ds + w_3 \cdot \int\limits_{0}^{d_i} (f_i^{\prime\prime\prime})^2(s) ds \Big)
$$
```

然后，Apollo定义$cost_2$表示最后的S-T路径和S-T巡航路径（有速度限制且m个点）的差值：

```
$$
cost_2 = \sum_{i=1}^{n}\sum_{j=1}^{m}\Big(f_i(t_j)- s_j\Big)^2
$$
```

同样地，Apollo定义了$cost_3$表示第一个S-T路径和随后的S-T路径（o个点）的差值：

```
$$
cost_3 = \sum_{i=1}^{n}\sum_{j=1}^{o}\Big(f_i(t_j)- s_j\Big)^2
$$
```

最后得出的目标函数为：

```
$$
cost = cost_1 + cost_2 + cost_3
$$
```

## 3  约束条件  

### 3.1 初始点约束

假设第一个点是($t0$, $s0$)，且$s0$在路径$f_i(t)$, $f'i(t)$, 和$f_i(t)''$上（位置、速率、加速度）。Apollo将这些约束转换为QP约束的等式为：

```
$$
A_{eq}x = b_{eq}
$$
```

### 3.2  单调约束

路线必须是单调的，比如车辆只能往前开。

在路径上采样 **m** 个点，对每一个 $j$和$j-1$ 的点对，且($j\in[1,...,m]$)，如果两个点都处在同一个样条$k$上，则：

```
$$
\begin{vmatrix}  1 & t_j & t_j^2 & t_j^3 & t_j^4&t_j^5 \\ \end{vmatrix} 
\cdot 
\begin{vmatrix}  a_k \\ b_k \\ c_k \\ d_k \\ e_k \\ f_k  \end{vmatrix} 
> 
\begin{vmatrix}  1 & t_{j-1} & t_{j-1}^2 & t_{j-1}^3 & t_{j-1}^4&t_{j-1}^5 \\ \end{vmatrix}  
\cdot 
\begin{vmatrix}  a_{k} \\ b_{k} \\ c_{k} \\ d_{k} \\ e_{k} \\ f_{k}  \end{vmatrix}
$$
```

如两个点分别处在不同的样条$k$和$l$上，则：

```
$$
\begin{vmatrix}  1 & t_j & t_j^2 & t_j^3 & t_j^4&t_j^5 \\ \end{vmatrix} 
\cdot 
\begin{vmatrix}  a_k \\ b_k \\ c_k \\ d_k \\ e_k \\ f_k  \end{vmatrix} 
> 
\begin{vmatrix}  1 & t_{j-1} & t_{j-1}^2 & t_{j-1}^3 & t_{j-1}^4&t_{j-1}^5 \\ \end{vmatrix}  
\cdot 
\begin{vmatrix}  a_{l} \\ b_{l} \\ c_{l} \\ d_{l} \\ e_{l} \\ f_{l}  \end{vmatrix}
$$
```

### 3.3  平滑节点约束

该约束的目的是使样条的节点更加平滑。假设两个段$seg_k$ 和$seg_{k+1}$互相连接，且$seg_k$的累计值 **s** 为$s_k$。计算约束的等式为：

```
$$
f_k(t_k) = f_{k+1} (t_0)
$$
```

即：
```
$$
\begin{vmatrix} 
 1 & t_k & t_k^2 & t_k^3 & t_k^4&t_k^5 \\
 \end{vmatrix} 
 \cdot 
 \begin{vmatrix} 
 a_{k0} \\ a_{k1} \\ a_{k2} \\ a_{k3} \\ a_{k4} \\ a_{k5} 
 \end{vmatrix} 
 = 
\begin{vmatrix} 
 1 & t_{0} & t_{0}^2 & t_{0}^3 & t_{0}^4&t_{0}^5 \\
 \end{vmatrix} 
 \cdot 
 \begin{vmatrix} 
 a_{k+1,0} \\ a_{k+1,1} \\ a_{k+1,2} \\ a_{k+1,3} \\ a_{k+1,4} \\ a_{k+1,5} 
 \end{vmatrix}
$$
```

然后，
```
$$
\begin{vmatrix} 
 1 & t_k & t_k^2 & t_k^3 & t_k^4&t_k^5 &  -1 & -t_{0} & -t_{0}^2 & -t_{0}^3 & -t_{0}^4&-t_{0}^5\\
 \end{vmatrix} 
 \cdot 
 \begin{vmatrix} 
 a_{k0} \\ a_{k1} \\ a_{k2} \\ a_{k3} \\ a_{k4} \\ a_{k5} \\ a_{k+1,0} \\ a_{k+1,1} \\ a_{k+1,2} \\ a_{k+1,3} \\ a_{k+1,4} \\ a_{k+1,5}   
 \end{vmatrix} 
 = 0
$$
```

等式中得出的结果为$t_0$ = 0。

同样地，为下述等式计算约束等式：

```
$$
f'_k(t_k) = f'_{k+1} (t_0)
\\
f''_k(t_k) = f''_{k+1} (t_0)
\\
f'''_k(t_k) = f'''_{k+1} (t_0)
$$
```

### 3.4  点采样边界约束

在路径上均匀的取样 **m** 个点，检查这些点上的障碍物边界。将这些约束转换为QP约束不等式，使用不等式：

```
$$
Ax \leq b
$$
```

首先基于道路宽度和周围的障碍物找到点 $(s_j, l_j)$的下边界$l_{lb,j}$，且$j\in[0, m]$。计算约束的不等式为：

```
$$
\begin{vmatrix} 
 1 & t_0 & t_0^2 & t_0^3 & t_0^4&t_0^5 \\
  1 & t_1 & t_1^2 & t_1^3 & t_1^4&t_1^5 \\
 ...&...&...&...&...&... \\
 1 & t_m & t_m^2 & t_m^3 & t_m^4&t_m^5 \\
 \end{vmatrix} \cdot \begin{vmatrix} a_i \\ b_i \\ c_i \\ d_i \\ e_i \\ f_i \end{vmatrix} 
 \leq 
 \begin{vmatrix}
 l_{lb,0}\\
 l_{lb,1}\\
 ...\\
 l_{lb,m}\\
 \end{vmatrix}
$$
```

同样地，对上边界$l_{ub,j}$，计算约束的不等式为：

```
$$
\begin{vmatrix} 
 1 & t_0 & t_0^2 & t_0^3 & t_0^4&t_0^5 \\
  1 & t_1 & t_1^2 & t_1^3 & t_1^4&t_1^5 \\
 ...&...&...&...&...&... \\
 1 & t_m & t_m^2 & t_m^3 & t_m^4&t_m^5 \\
 \end{vmatrix} \cdot \begin{vmatrix} a_i \\ b_i \\ c_i \\ d_i \\ e_i \\ f_i \end{vmatrix} 
 \leq
 -1 \cdot
 \begin{vmatrix}
 l_{ub,0}\\
 l_{ub,1}\\
 ...\\
 l_{ub,m}\\
 \end{vmatrix}
$$
```

### 3.5  速度边界优化

Apollo同样需要建立速度限制边界。

在st曲线上取样 **m** 个点，为每个点$j$获取速度限制的上边界和下边界，例如$v{ub,j}$ 和 $v{lb,j}$，约束定义为：

```
$$
f'(t_j) \geq v_{lb,j}
$$
```
即：
```
$$
\begin{vmatrix}  
0& 1 & t_0 & t_0^2 & t_0^3 & t_0^4 \\  
0 & 1 & t_1 & t_1^2 & t_1^3 & t_1^4 \\ 
...&...&...&...&...&... \\ 
0& 1 & t_m & t_m^2 & t_m^3 & t_m^4 \\ 
\end{vmatrix} 
\cdot 
\begin{vmatrix} 
a_i \\ b_i \\ c_i \\ d_i \\ e_i \\ f_i 
\end{vmatrix}  
\geq  
\begin{vmatrix} v_{lb,0}\\ v_{lb,1}\\ ...\\ v_{lb,m}\\ \end{vmatrix}
$$
```
且，
```
$$
f'(t_j) \leq v_{ub,j}
$$
```
即：
```
$$
\begin{vmatrix} 
 0& 1 & t_0 & t_0^2 & t_0^3 & t_0^4 \\
 0 & 1 & t_1 & t_1^2 & t_1^3 & t_1^4 \\
 ...&...&...&...&...&... \\
 0 &1 & t_m & t_m^2 & t_m^3 & t_m^4 \\
 \end{vmatrix} \cdot \begin{vmatrix} a_i \\ b_i \\ c_i \\ d_i \\ e_i \\ f_i \end{vmatrix} 
 \leq
 \begin{vmatrix}
 v_{ub,0}\\
 v_{ub,1}\\
 ...\\
 v_{ub,m}\\
 \end{vmatrix}
$$
```
