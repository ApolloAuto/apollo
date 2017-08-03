# QP-Spline-Path Optimizer

Quadratic programming + Spline interpolation

## Step1: Get path length 

Path is defined in station-lateral corrdination system. The **s** range from veichle's current position to  default planing path length.

## Step2:  Get spline segments

Split the path into n segments. each segment trajectory is defined b a polynomial. 

## Step 3: define function for each spline segment

Each segment *i* has accumulated distance $d_i$ along reference line. And the trajactory for the segment is defined as a  polynomial of degree five by default

$$
l = f_i(s) 
  = a_i + b_i * s + c_i * s^2 + d_i * s^3 + e_i * s^4 + f_i * s^5   (0 \leq s \leq d_{i})
$$

## Step 4: Define objective function of optimization for each segment

$$
cost = \sum_{i=1}^{n} \Big( w_1 \cdot \int\limits_{0}^{d_i} (f_i')^2(s) ds + w_2 \cdot \int\limits_{0}^{d_i} (f_i'')^2(s) ds + w_3 \cdot \int\limits_{0}^{d_i} (f_i^{\prime\prime\prime})^2(s) ds \Big)
$$

## Step 5: Convert the cost function to QP formulation

QP formulation:
$$
\frac{1}{2} \cdot x^T \cdot H \cdot x + f^T \cdot x 
\\
s.t. LB \leq x \leq UB
\\
A_{eq}x = b_{eq}
\\
Ax \leq b
$$
Below is the example for converting cost function to QP formulaiton. 
$$
f_i(s) ＝\begin{vmatrix} a_i & b_i & c_i & d_i & e_i & f_i \end{vmatrix} \cdot  \begin{vmatrix} 1 \\ s \\ s^2 \\ s^3 \\ s^4 \\ s^5 \end{vmatrix}
$$

and
$$
f_i'(s) =\begin{vmatrix} a_i & b_i & c_i & d_i & e_i & f_i \end{vmatrix} \cdot  \begin{vmatrix} 0 \\ 1 \\ s \\ s^2 \\ s^3 \\ s^4 \end{vmatrix}
$$


and 
$$
f_i'(s)^2 =\begin{vmatrix} a_i & b_i & c_i & d_i & e_i & f_i \end{vmatrix} \cdot  \begin{vmatrix} 0 \\ 1 \\ s \\ s^2 \\ s^3 \\ s^4 \end{vmatrix} \cdot \begin{vmatrix} 0 & 1 & s & s^2 & s^3 & s^4 \end{vmatrix} \cdot \begin{vmatrix} a_i \\ b_i \\ c_i \\ d_i \\ e_i \\ f_i \end{vmatrix}
$$
and 
$$
\int\limits_{0}^{d_i} f_i'(s)^2 ds ＝\int\limits_{0}^{d_i}\begin{vmatrix} a_i & b_i & c_i & d_i & e_i & f_i \end{vmatrix} \cdot  \begin{vmatrix} 0 \\ 1 \\ s \\ s^2 \\ s^3 \\ s^4 \end{vmatrix} \cdot \begin{vmatrix} 0 & 1 & s & s^2 & s^3 & s^4 \end{vmatrix} \cdot \begin{vmatrix} a_i \\ b_i \\ c_i \\ d_i \\ e_i \\ f_i \end{vmatrix}
$$


and
$$
\int\limits_{0}^{d_i} f'(s)^2 ds ＝\begin{vmatrix} a_i & b_i & c_i & d_i & e_i & f_i \end{vmatrix} \cdot \int\limits_{0}^{d_i}  \begin{vmatrix} 0 \\ 1 \\ s \\ s^2 \\ s^3 \\ s^4 \end{vmatrix} \cdot \begin{vmatrix} 0 & 1 & s & s^2 & s^3 & s^4 \end{vmatrix} ds \cdot \begin{vmatrix} a_i \\ b_i \\ c_i \\ d_i \\ e_i \\ f_i \end{vmatrix}
$$


and
$$
\int\limits_{0}^{d_i} 
f'(s)^2 ds ＝\begin{vmatrix} a_i & b_i & c_i & d_i & e_i & f_i \end{vmatrix} 
\cdot \int\limits_{0}^{d_i}
\begin{vmatrix} 
0  & 0 &0&0&0&0\\ 
0 & 1 & s & s^2 & s^3 & s^4\\  
0 & s & s^2 & s^3 & s^4 & s^5\\ 
0 & s^2 &  s^3 & s^4&s^5&s^6 \\ 
0 & s^3 & s^4 &s^5 &s^6&s^7 \\ 
0 & s^4 & s^5 & s^6 & s^7 & s^8 
\end{vmatrix} ds \cdot \begin{vmatrix} a_i \\ b_i \\ c_i \\ d_i \\ e_i \\ f_i \end{vmatrix}
$$
and
$$
\int\limits_{0}^{d_i} 
f'_i(s)^2 ds ＝\begin{vmatrix} a_i & b_i & c_i & d_i & e_i & f_i \end{vmatrix} 
\cdot \begin{vmatrix} 
0 & 0 & 0 & 0 &0&0\\ 
0 & d_i & \frac{d_i^2}{2} & \frac{d_i^3}{3} & \frac{d_i^4}{4}&\frac{d_i^5}{5}\\ 
0& \frac{d_i^2}{2} & \frac{d_i^3}{3} & \frac{d_i^4}{4} & \frac{d_i^5}{5}&\frac{d_i^6}{6}\\ 
0& \frac{d_i^3}{3} & \frac{d_i^4}{4} & \frac{d_i^5}{5} & \frac{d_i^6}{6}&\frac{d_i^7}{7}\\ 
0& \frac{d_i^4}{4} & \frac{d_i^5}{5} & \frac{d_i^6}{6} & \frac{d_i^7}{7}&\frac{d_i^8}{8}\\ 
0& \frac{d_i^5}{5} & \frac{d_i^6}{6} & \frac{d_i^7}{7} & \frac{d_i^8}{8}&\frac{d_i^9}{9}\\ 
\end{vmatrix} 
\cdot 
\begin{vmatrix} a_i \\ b_i \\ c_i \\ d_i \\ e_i \\ f_i \end{vmatrix}
$$

## Step 5: Constraint  

