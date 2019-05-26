# Reference Line Smoother

_**Tip**: to read the equations in the document, you are recommended to use Chrome with [a plugin](https://chrome.google.com/webstore/detail/tex-all-the-things/cbimabofgmfdkicghcadidpemeenbffn) or copy the latex equation to [an online editor](http://www.hostmath.com/)_

Quadratic programming + Spline interpolation

## 1. Objective function

### 1.1 Segment routing path

Segment routing path into **n** segments. each segment trajectory is defined by two polynomials:

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

### 1.2 Define objective function of optimization for each segment

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

### 1.3 Convert the cost function to QP formulation

QP formulation:
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


## 2 Constraints


### 2.1 Joint smoothness  constraints

This constraint smoothes the spline joint.  Let's assume two segments, $seg_k$ and $seg_{k+1}$, are connected and the accumulated **s** of segment $seg_k$ is $s_k$. Calculate the constraint equation as:
```
$$
f_k(s_k) = f_{k+1} (s_0)
$$
```
Similarly the formula works for the equality constraints, such as:
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

### 2.2 Sampled points for boundary constraint

Evenly sample **m** points along the path and check the predefined boundaries at those points.
```
$$
f_i(t_l) - x_l< boundary
\\
g_i(t_l) - y_l< boundary
$$
```
