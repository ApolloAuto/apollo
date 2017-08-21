# QP-Spline-ST-Speed Optimizer



## 1 Definition 

After finding a path in QP-Spline-Path, we convert all obstacles on the path and ADV (autonomous driving vehicle) itself into a ST graph, which represents the stations changes over time along the path. Our speed optimization task is to find a path on ST graph that is collision-free and comfort. 

We use spline to define the path, and find the best path leveraging Quadratic programming with a set of conditions. QP formulation is defined as 
$$
\frac{1}{2} \cdot x^T \cdot H \cdot x + f^T \cdot x 
\\
s.t. LB \leq x \leq UB
\\
A_{eq}x = b_{eq}
\\
Ax \leq b
$$

## 2 Objective function

### 1.1  Get spline segments

Split the path into n segments. each segment trajectory is defined by a polynomial.

### 1.2 Define function for each spline segment

Each segment *i* has accumulated distance $d_i$ along reference line. And the trajactory for the segment is defined as a polynomial of degree five by default

$$
s = f_i(t) 
  = a_{0i} + a_{1i} \cdot t + a_{2i} \cdot t^2 + a_{3i} \cdot t^3 + a_{4i} \cdot t^4 + a_{5i} \cdot t^5
$$

### 1.3 Define objective function of optimization for each segment

We first define $cost_1$ to make the trajectory smooth 
$$
cost_1 = \sum_{i=1}^{n} \Big( w_1 \cdot \int\limits_{0}^{d_i} (f_i')^2(s) ds + w_2 \cdot \int\limits_{0}^{d_i} (f_i'')^2(s) ds + w_3 \cdot \int\limits_{0}^{d_i} (f_i^{\prime\prime\prime})^2(s) ds \Big)
$$

Then we define $cost_2$ that is the difference between the final ST trajectory and cruise ST trajectory (with given speed limits) (m points)
$$
cost_2 = \sum_{i=1}^{n}\sum_{j=1}^{m}\Big(f_i(t_j)- s_j\Big)^2
$$
Similarly, we define $cost_3$ that is the difference between the finial ST path and the follow ST path (o points)
$$
cost_3 = \sum_{i=1}^{n}\sum_{j=1}^{o}\Big(f_i(t_j)- s_j\Big)^2
$$
Finally, the objective function is defined as 
$$
cost = cost_1 + cost_2 + cost_3
$$



## 2 Constraints  

### 2.1 The init point constraints

Let's assume the the first point is ($t_0$, $s_0$), and $s_0$ should be on the planned path $f_i(t)$, $f'_i(t)$, and $f_i(t)''$ (position, velocity, acceleration).  we convert those constraint into QP equality constraints 
$$
A_{eq}x = b_{eq}
$$

### 2.2 Monotone constraint

The path need to be monotone, e.g., vehicle can only drive forward. 

sample m points on the path, for each $j$ and $j-1$ point pairs ($j\in[1,...,m]$) 

if the two pionts on the same spline $k​$
$$
\begin{vmatrix}  1 & t_j & t_j^2 & t_j^3 & t_j^4&t_j^5 \\ \end{vmatrix} 
\cdot 
\begin{vmatrix}  a_k \\ b_k \\ c_k \\ d_k \\ e_k \\ f_k  \end{vmatrix} 
> 
\begin{vmatrix}  1 & t_{j-1} & t_{j-1}^2 & t_{j-1}^3 & t_{j-1}^4&t_{j-1}^5 \\ \end{vmatrix}  
\cdot 
\begin{vmatrix}  a_{k} \\ b_{k} \\ c_{k} \\ d_{k} \\ e_{k} \\ f_{k}  \end{vmatrix}
$$
 if the two points on the different spline $k$ and $l$
$$
\begin{vmatrix}  1 & t_j & t_j^2 & t_j^3 & t_j^4&t_j^5 \\ \end{vmatrix} 
\cdot 
\begin{vmatrix}  a_k \\ b_k \\ c_k \\ d_k \\ e_k \\ f_k  \end{vmatrix} 
> 
\begin{vmatrix}  1 & t_{j-1} & t_{j-1}^2 & t_{j-1}^3 & t_{j-1}^4&t_{j-1}^5 \\ \end{vmatrix}  
\cdot 
\begin{vmatrix}  a_{l} \\ b_{l} \\ c_{l} \\ d_{l} \\ e_{l} \\ f_{l}  \end{vmatrix}
$$



### 2.3 Joint smoothness  constraints

This constraint is to make the spline joint smooth.  Let's assume two segment $seg_k$ and $seg_{k+1}$ are connected, and the accumulated s of segment $seg_k$ is $s_k$. The we can get the constraint euqation as 
$$
f_k(t_k) = f_{k+1} (t_0)
$$
namely
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
then
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
$t_0$ = 0 in the equation.

Similarly we can get the equality constraints for 
$$
f'_k(t_k) = f'_{k+1} (t_0)
\\
f''_k(t_k) = f''_{k+1} (t_0)
\\
f'''_k(t_k) = f'''_{k+1} (t_0)
$$

### 2.4 Sampled points for boundary constraint

Evenly sample m points along the path, and check the obstacle boundary at those points.  convert the constraint into QP inequality constraints
$$
Ax \leq b
$$
We first find lower boundary $l_{lb,j}$ at those points ($s_j$, $l_j$) and  $j\in[0, m]$ based on road width and surrounding obstacles. The calculate the inequality constraints as
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


similarly, for upper boundary $l_{ub,j}$, we get the ineuqality constraints as 
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



### 2.5 Speed Boundary constraint

We need to limit speed boundary as well.

Sample m points on st curve, and get speed limits upper bound and lower bound for each point $j$, e.g., $v_{ub,j}$ and $v_{lb,j}$ . The constraints is defined as 
$$
f'(t_j) \geq v_{lb,j}
$$
namely
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
and 
$$
f'(t_j) \leq v_{ub,j}
$$
namely
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
