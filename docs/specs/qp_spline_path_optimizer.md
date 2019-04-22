# QP-Spline-Path Optimizer

_**Tip**: to read the equations in the document, you are recommended to use Chrome with [a plugin](https://chrome.google.com/webstore/detail/tex-all-the-things/cbimabofgmfdkicghcadidpemeenbffn) or copy the latex equation to [an online editor](http://www.hostmath.com/)_

Quadratic programming + Spline interpolation

## 1.  Objective function

### 1.1  Get path length

Path is defined in station-lateral coordination system. The **s** range from vehicle's current position to  default planning path length.

### 1.2   Get spline segments

Split the path into **n** segments. each segment trajectory is defined by a polynomial.

### 1.3  Define function for each spline segment

Each segment ***i*** has accumulated distance $d_i$ along reference line. The trajectory for the segment is defined as a polynomial of degree five by default.

```
$$
l = f_i(s)
  = a_{i0} + a_{i1} \cdot s + a_{i2} \cdot s^2 + a_{i3} \cdot s^3 + a_{i4} \cdot s^4 + a_{i5} \cdot s^5        (0 \leq s \leq d_{i})
$$
```

### 1.4  Define objective function of optimization for each segment

```
$$
cost = \sum_{i=1}^{n} \Big( w_1 \cdot \int\limits_{0}^{d_i} (f_i')^2(s) ds + w_2 \cdot \int\limits_{0}^{d_i} (f_i'')^2(s) ds + w_3 \cdot \int\limits_{0}^{d_i} (f_i^{\prime\prime\prime})^2(s) ds \Big)
$$
```

### 1.5  Convert the cost function to QP formulation

QP formulation:
```
$$
\begin{aligned}
minimize  & \frac{1}{2}  \cdot x^T \cdot H \cdot x  + f^T \cdot x \\
s.t. \qquad & LB \leq x \leq UB \\
      & A_{eq}x = b_{eq} \\
      & Ax \geq b
\end{aligned}
$$
```
Below is the example for converting the cost function into the QP formulation. 
```
$$
f_i(s) ＝
\begin{vmatrix} 1 & s & s^2 & s^3 & s^4 & s^5 \end{vmatrix}
\cdot
\begin{vmatrix} a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5} \end{vmatrix}   
$$
```

And
```
$$
f_i'(s) =
\begin{vmatrix} 0 & 1 & 2s & 3s^2 & 4s^3 & 5s^4 \end{vmatrix}
\cdot
\begin{vmatrix} a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5} \end{vmatrix}   
$$
```


And 
```
$$
f_i'(s)^2 =
\begin{vmatrix} a_{i0} & a_{i1} & a_{i2} & a_{i3} & a_{i4} & a_{i5}  \end{vmatrix} 
\cdot 
\begin{vmatrix} 0 \\ 1 \\ 2s \\ 3s^2 \\ 4s^3 \\ 5s^4 \end{vmatrix} 
\cdot 
\begin{vmatrix} 0 & 1 & 2s & 3s^2 & 4s^3 & 5s^4 \end{vmatrix} 
\cdot 
\begin{vmatrix} a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5}  \end{vmatrix}
$$
```
then we have,
```
$$
\int\limits_{0}^{d_i} f_i'(s)^2 ds ＝
\int\limits_{0}^{d_i}
\begin{vmatrix} a_{i0} & a_{i1} & a_{i2} & a_{i3} & a_{i4} & a_{i5} \end{vmatrix} 
\cdot  
\begin{vmatrix} 0 \\ 1 \\ 2s \\ 3s^2 \\ 4s^3 \\ 5s^4 \end{vmatrix} 
\cdot 
\begin{vmatrix} 0 & 1 & 2s & 3s^2 & 4s^3 & 5s^4 \end{vmatrix} 
\cdot 
\begin{vmatrix} a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5}  \end{vmatrix} ds
$$
```


extract the const outside the integration, we have,
```
$$
\int\limits_{0}^{d_i} f'(s)^2 ds ＝
\begin{vmatrix} a_{i0} & a_{i1} & a_{i2} & a_{i3} & a_{i4} & a_{i5} \end{vmatrix} 
\cdot 
\int\limits_{0}^{d_i}  
\begin{vmatrix} 0 \\ 1 \\ 2s \\ 3s^2 \\ 4s^3 \\ 5s^4 \end{vmatrix} 
\cdot 
\begin{vmatrix} 0 & 1 & 2s & 3s^2 & 4s^3 & 5s^4 \end{vmatrix} ds 
\cdot 
\begin{vmatrix} a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5}  \end{vmatrix}
$$
$$
＝\begin{vmatrix} a_{i0} & a_{i1} & a_{i2} & a_{i3} & a_{i4} & a_{i5} \end{vmatrix} 
\cdot \int\limits_{0}^{d_i}
\begin{vmatrix} 
0  & 0 &0&0&0&0\\ 
0 & 1 & 2s & 3s^2 & 4s^3 & 5s^4\\
0 & 2s & 4s^2 & 6s^3 & 8s^4 & 10s^5\\
0 & 3s^2 &  6s^3 & 9s^4 & 12s^5&15s^6 \\
0 & 4s^3 & 8s^4 &12s^5 &16s^6&20s^7 \\
0 & 5s^4 & 10s^5 & 15s^6 & 20s^7 & 25s^8 
\end{vmatrix} ds 
\cdot 
\begin{vmatrix} a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5} \end{vmatrix}
$$
```

Finally, we have

```
$$
\int\limits_{0}^{d_i} 
f'_i(s)^2 ds =\begin{vmatrix} a_{i0} & a_{i1} & a_{i2} & a_{i3} & a_{i4} & a_{i5} \end{vmatrix} 
\cdot \begin{vmatrix} 
0 & 0 & 0 & 0 &0&0\\ 
0 & d_i & d_i^2 & d_i^3 & d_i^4&d_i^5\\
0& d_i^2 & \frac{4}{3}d_i^3& \frac{6}{4}d_i^4 & \frac{8}{5}d_i^5&\frac{10}{6}d_i^6\\
0& d_i^3 & \frac{6}{4}d_i^4 & \frac{9}{5}d_i^5 & \frac{12}{6}d_i^6&\frac{15}{7}d_i^7\\
0& d_i^4 & \frac{8}{5}d_i^5 & \frac{12}{6}d_i^6 & \frac{16}{7}d_i^7&\frac{20}{8}d_i^8\\
0& d_i^5 & \frac{10}{6}d_i^6 & \frac{15}{7}d_i^7 & \frac{20}{8}d_i^8&\frac{25}{9}d_i^9
\end{vmatrix} 
\cdot 
\begin{vmatrix} a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5} \end{vmatrix}
$$
```

Please notice that we got a 6 x 6 matrix to represent the derivative cost of 5th order spline.



Similar deduction can also be used to calculate the cost of second and third order derivatives.



## 2  Constraints  

### 2.1  The init point constraints

Assume that the first point is ($s_0$, $l_0$), ($s_0$, $l'_0$) and ($s_0$, $l''_0$), where $l_0$ , $l'_0$ and $l''_0$ is the lateral offset and its first and second derivatives on the init point of planned path, and are calculated from $f_i(s)$, $f'_i(s)$, and $f_i(s)''$.  

Convert those constraints into QP equality constraints, using: 
```
$$
A_{eq}x = b_{eq}
$$
```
Below are the steps of conversion.
```
$$
f_i(s_0) = 
\begin{vmatrix} 1 & s_0 & s_0^2 & s_0^3 & s_0^4&s_0^5 \end{vmatrix} 
\cdot 
\begin{vmatrix}  a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5}\end{vmatrix} = l_0
$$
```
And
```
$$
f'_i(s_0) = 
\begin{vmatrix} 0& 1 & 2s_0 & 3s_0^2 & 4s_0^3 &5 s_0^4 \end{vmatrix} 
\cdot 
\begin{vmatrix}  a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5} \end{vmatrix} = l'_0
$$
```
And 
```
$$
f''_i(s_0) = 
\begin{vmatrix} 0&0& 2 & 3\times2s_0 & 4\times3s_0^2 & 5\times4s_0^3  \end{vmatrix} 
\cdot 
\begin{vmatrix}  a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5} \end{vmatrix} = l''_0
$$
```
where i is the index of the segment that contains the $s_0$.

### 2.2  The end point constraints

Similar to the init point, the end point $(s_e, l_e)$ is known and should produce the same constraint as described in the init point calculations. 


Combine the init point and end point, and show the equality constraint as: 

```
$$
\begin{vmatrix} 
 1 & s_0 & s_0^2 & s_0^3 & s_0^4&s_0^5 \\
 0&1 & 2s_0 & 3s_0^2 & 4s_0^3 & 5s_0^4 \\
 0& 0&2 & 3\times2s_0 & 4\times3s_0^2 & 5\times4s_0^3  \\
 1 & s_e & s_e^2 & s_e^3 & s_e^4&s_e^5 \\
 0&1 & 2s_e & 3s_e^2 & 4s_e^3 & 5s_e^4 \\
 0& 0&2 & 3\times2s_e & 4\times3s_e^2 & 5\times4s_e^3  
 \end{vmatrix} 
 \cdot 
 \begin{vmatrix}  a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5} \end{vmatrix} 
 = 
 \begin{vmatrix}
 l_0\\
 l'_0\\
 l''_0\\
 l_e\\
 l'_e\\
 l''_e\\
 \end{vmatrix}
$$
```

### 2.3  Joint smoothness  constraints

This constraint is designed to smooth the spline joint.  Assume two segments $seg_k$ and $seg_{k+1}$ are connected, and the accumulated **s** of segment $seg_k$ is $s_k$. Calculate the constraint equation as: 
```
$$
f_k(s_k) = f_{k+1} (s_0)
$$
```
Below are the steps of the calculation.
```
$$
\begin{vmatrix} 
 1 & s_k & s_k^2 & s_k^3 & s_k^4&s_k^5 \\
 \end{vmatrix} 
 \cdot 
 \begin{vmatrix} 
 a_{k0} \\ a_{k1} \\ a_{k2} \\ a_{k3} \\ a_{k4} \\ a_{k5} 
 \end{vmatrix} 
 = 
\begin{vmatrix} 
 1 & s_{0} & s_{0}^2 & s_{0}^3 & s_{0}^4&s_{0}^5 \\
 \end{vmatrix} 
 \cdot 
 \begin{vmatrix} 
 a_{k+1,0} \\ a_{k+1,1} \\ a_{k+1,2} \\ a_{k+1,3} \\ a_{k+1,4} \\ a_{k+1,5} 
 \end{vmatrix}
$$
```
Then
```
$$
\begin{vmatrix} 
 1 & s_k & s_k^2 & s_k^3 & s_k^4&s_k^5 &  -1 & -s_{0} & -s_{0}^2 & -s_{0}^3 & -s_{0}^4&-s_{0}^5\\
 \end{vmatrix} 
 \cdot 
 \begin{vmatrix} 
 a_{k0} \\ a_{k1} \\ a_{k2} \\ a_{k3} \\ a_{k4} \\ a_{k5} \\ a_{k+1,0} \\ a_{k+1,1} \\ a_{k+1,2} \\ a_{k+1,3} \\ a_{k+1,4} \\ a_{k+1,5}  
 \end{vmatrix} 
 = 0
$$
```
Use $s_0$ = 0 in the equation.

Similarly calculate the equality constraints for: 
```
$$
f'_k(s_k) = f'_{k+1} (s_0)
\\
f''_k(s_k) = f''_{k+1} (s_0)
\\
f'''_k(s_k) = f'''_{k+1} (s_0)
$$
```

### 2.4  Sampled points for boundary constraint

Evenly sample **m** points along the path, and check the obstacle boundary at those points.  Convert the constraint into QP inequality constraints, using:
```
$$
Ax \geq b
$$
```
First find the lower boundary $l_{lb,j}$ at those points $(s_j, l_j)$ and  $j\in[0, m]$ based on the road width and surrounding obstacles. Calculate the inequality constraints as:
```
$$
\begin{vmatrix} 
 1 & s_0 & s_0^2 & s_0^3 & s_0^4&s_0^5 \\
  1 & s_1 & s_1^2 & s_1^3 & s_1^4&s_1^5 \\
 ...&...&...&...&...&... \\
 1 & s_m & s_m^2 & s_m^3 & s_m^4&s_m^5 \\
 \end{vmatrix} \cdot \begin{vmatrix}a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5}  \end{vmatrix} 
 \geq 
 \begin{vmatrix}
 l_{lb,0}\\
 l_{lb,1}\\
 ...\\
 l_{lb,m}\\
 \end{vmatrix}
$$
```


Similarly, for the upper boundary $l_{ub,j}$, calculate the inequality constraints as: 
```
$$
\begin{vmatrix} 
 -1 & -s_0 & -s_0^2 & -s_0^3 & -s_0^4&-s_0^5 \\
  -1 & -s_1 & -s_1^2 & -s_1^3 & -s_1^4&-s_1^5 \\
 ...&...-&...&...&...&... \\
 -1 & -s_m & -s_m^2 & -s_m^3 & -s_m^4&-s_m^5 \\
 \end{vmatrix} 
 \cdot 
 \begin{vmatrix} a_{i0} \\ a_{i1} \\ a_{i2} \\ a_{i3} \\ a_{i4} \\ a_{i5}  \end{vmatrix} 
 \geq
 -1 \cdot
 \begin{vmatrix}
 l_{ub,0}\\
 l_{ub,1}\\
 ...\\
 l_{ub,m}\\
 \end{vmatrix}
$$
```

