/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include "../core/i_blas.h"

#define I_DEFAULT_MAX_POLY_STURMCHAIN_ITERATIONS 50
#define I_DEFAULT_MAX_POLY_BISECTION_ITERATIONS 50

namespace idl {
/*Evaluate the n-th degree polynomial p at point x*/
template <typename T>
inline T i_poly_eval(const T *p, int n, T x) {
  T val = p[0];
  T y = x;
  for (int i = 1; i <= n; y *= x, i++) {
    val += (p[i] * y);
  }
  return val;
}
template <typename T>
inline T i_poly_eval1(const T p[2], T x) {
  return (p[0] + p[1] * x);
}
template <typename T>
inline T i_poly_eval2(const T p[3], T x) {
  return (p[0] + p[1] * x + p[2] * x * x);
}
template <typename T>
inline T i_poly_eval3(const T p[4], T x) {
  T x2 = x * x;
  T x3 = x2 * x;
  return (p[0] + p[1] * x + p[2] * x2 + p[3] * x3);
}
template <typename T>
inline T i_poly_eval4(const T p[5], T x) {
  T x2 = x * x;
  T x3 = x2 * x;
  T x4 = x3 * x;
  return (p[0] + p[1] * x + p[2] * x2 + p[3] * x3 + p[4] * x4);
}
template <typename T>
inline T i_poly_eval5(const T p[6], T x) {
  T x2 = x * x;
  T x3 = x2 * x;
  T x4 = x3 * x;
  T x5 = x4 * x;
  return (p[0] + p[1] * x + p[2] * x2 + p[3] * x3 + p[4] * x4 + p[5] * x5);
}
template <typename T>
inline T i_poly_eval6(const T p[7], T x) {
  T x2 = x * x;
  T x3 = x2 * x;
  T x4 = x3 * x;
  T x5 = x4 * x;
  T x6 = x5 * x;
  return (p[0] + p[1] * x + p[2] * x2 + p[3] * x3 + p[4] * x4 + p[5] * x5 +
          p[6] * x6);
}
template <typename T>
inline T i_poly_eval7(const T p[8], T x) {
  T x2 = x * x;
  T x3 = x2 * x;
  T x4 = x3 * x;
  T x5 = x4 * x;
  T x6 = x5 * x;
  T x7 = x6 * x;
  return (p[0] + p[1] * x + p[2] * x2 + p[3] * x3 + p[4] * x4 + p[5] * x5 +
          p[6] * x6 + p[7] * x7);
}
template <typename T>
inline T i_poly_eval8(const T p[9], T x) {
  T x2 = x * x;
  T x3 = x2 * x;
  T x4 = x3 * x;
  T x5 = x4 * x;
  T x6 = x5 * x;
  T x7 = x6 * x;
  T x8 = x7 * x;
  return (p[0] + p[1] * x + p[2] * x2 + p[3] * x3 + p[4] * x4 + p[5] * x5 +
          p[6] * x6 + p[7] * x7 + p[8] * x8);
}
template <typename T>
inline T i_poly_eval9(const T p[10], T x) {
  T x2 = x * x;
  T x3 = x2 * x;
  T x4 = x3 * x;
  T x5 = x4 * x;
  T x6 = x5 * x;
  T x7 = x6 * x;
  T x8 = x7 * x;
  T x9 = x8 * x;
  return (p[0] + p[1] * x + p[2] * x2 + p[3] * x3 + p[4] * x4 + p[5] * x5 +
          p[6] * x6 + p[7] * x7 + p[8] * x8 + p[9] * x9);
}
template <typename T>
inline T i_poly_eval10(const T p[11], T x) {
  T x2 = x * x;
  T x3 = x2 * x;
  T x4 = x3 * x;
  T x5 = x4 * x;
  T x6 = x5 * x;
  T x7 = x6 * x;
  T x8 = x7 * x;
  T x9 = x8 * x;
  T x10 = x9 * x;
  return (p[0] + p[1] * x + p[2] * x2 + p[3] * x3 + p[4] * x4 + p[5] * x5 +
          p[6] * x6 + p[7] * x7 + p[8] * x8 + p[9] * x9 + p[10] * x10);
}

/*Compute the derivative of polynomial of degree n*/
template <typename T>
inline void i_poly_deriv(const T *p, T *dp, int n) {
  for (int i = 1; i <= n; i++) {
    dp[i - 1] = p[i] * i;
  }
}

/*Multiply polynomial p1 of degree n1 with polynomial p2 of degree n2
to make polynomial p1p2 of degree n1+n2*/
template <typename T>
inline void i_poly_mult(const T *p1, const T *p2, T *p1p2, int n1, int n2) {
  i_zero(p1p2, n1 + n2 + 1);
  for (int i = 0; i <= n1; i++) {
    for (int j = 0; j <= n2; j++) {
      p1p2[i + j] += p1[i] * p2[j];
    }
  }
}

template <typename T>
inline void i_poly_mult_1_1(const T a[2], const T b[2], T p[3]) {
  p[0] = a[0] * b[0];
  p[1] = a[0] * b[1] + a[1] * b[0];
  p[2] = a[1] * b[1];
}

template <typename T>
inline void i_poly_mult_1_2(const T a[2], const T b[3], T p[4]) {
  p[0] = a[0] * b[0];
  p[1] = a[0] * b[1] + a[1] * b[0];
  p[2] = a[0] * b[2] + a[1] * b[1];
  p[3] = a[1] * b[2];
}

template <typename T>
inline void i_add_poly_mult_1_1(const T a[2], const T b[2], T p[3]) {
  p[0] += a[0] * b[0];
  p[1] += a[0] * b[1] + a[1] * b[0];
  p[2] += a[1] * b[1];
}

template <typename T>
inline void i_add_poly_mult_1_2(const T a[2], const T b[3], T p[4]) {
  p[0] += a[0] * b[0];
  p[1] += a[0] * b[1] + a[1] * b[0];
  p[2] += a[0] * b[2] + a[1] * b[1];
  p[3] += a[1] * b[2];
}

template <typename T>
inline void i_sub_poly_mult_1_1(const T a[2], const T b[2], T p[3]) {
  p[0] -= a[0] * b[0];
  p[1] -= a[0] * b[1] + a[1] * b[0];
  p[2] -= a[1] * b[1];
}

template <typename T>
inline void i_sub_poly_mult_1_2(const T a[2], const T b[3], T p[4]) {
  p[0] -= a[0] * b[0];
  p[1] -= a[0] * b[1] + a[1] * b[0];
  p[2] -= a[0] * b[2] + a[1] * b[1];
  p[3] -= a[1] * b[2];
}

/*Divide polynomial p1 of degree n>=2 by polynomial p2 of degree n-1.
return the linear quotient in k and b (representing kx+b) and
the negation of the (n-2)th degree remainder in neg_r. it is assumed that the
leading coefficients of p1 and p2 are non-zero. this function is tailored for
computing sturm sequences*/
template <typename T>
inline void i_poly_div_one_degree_diff(const T *p1, const T *p2, T &k, T &b,
                                       T *neg_r, int n) {
  int m = n - 1;
  /*quotient*/
  k = i_div(p1[n], p2[m]);
  b = i_div(p1[m] - k * p2[n - 2], p2[m]);
  /*negation of remainder*/
  for (int i = 1; i < m; i++) {
    neg_r[i] = k * p2[i - 1] + b * p2[i] - p1[i];
  }
  neg_r[0] = b * p2[0] - p1[0];
}

/*Add the result of multiplying polynomial p1 of degree n1 with p2 of degree n2
to polynomial p1p2 of degree n1+n2*/
template <typename T>
inline void i_poly_mult_add(const T *p1, const T *p2, T *p1p2, int n1, int n2) {
  for (int i = 0; i <= n1; i++) {
    for (int j = 0; j <= n2; j++) {
      p1p2[i + j] += p1[i] * p2[j];
    }
  }
}

/*Subtract the result of multiplying polynomial p1 of degree n1 with p2 of
degree n2
from polynomial p1p2 of degree n1+n2*/
template <typename T>
inline void i_poly_mult_sub(const T *p1, const T *p2, T *p1p2, int n1, int n2) {
  for (int i = 0; i <= n1; i++) {
    for (int j = 0; j <= n2; j++) {
      p1p2[i + j] -= p1[i] * p2[j];
    }
  }
}

/*Compute a representation k,b for the Sturm chain of n-th degree polynomial p
(where n is assumed to be >=2).
the chain will need n+1 entries in both k and b.
the function should be provided 2*n entries of scratch space in d, in total 4n+2
spaces*/
template <typename T>
inline void i_construct_sturm_chain(const T *p, T *k, T *b, T *d, int n) {
  T *d1, *d2;
  d1 = d;
  d2 = d + n;
  k[0] = 0;
  /*compute derivative of polynomial and store results in d1*/
  i_poly_deriv(p, d1, n);
  /*compute remainder and quotient of division between polynomial p and its
   * derivative d1 and store result in d2*/
  i_poly_div_one_degree_diff(p, d1, k[n], b[n], d2, n);
  if (n < 3) {
    k[1] = d1[1];
    b[1] = d1[0];
    b[0] = d2[0];
    return;
  }
  for (int i = n - 1; i >= 2;) {
    /*compute remainder and quotient of division between d1 and d2 and store
     * result in d1*/
    i_poly_div_one_degree_diff(d1, d2, k[i], b[i], d1, i);
    i--;
    if (i < 2) {
      k[1] = d2[1];
      b[1] = d2[0];
      b[0] = d1[0];
      break;
    }
    /*compute remainder and quotient of division between d2 and d1 and store
     * result in d2*/
    i_poly_div_one_degree_diff(d2, d1, k[i], b[i], d2, i);
    i--;
    if (i < 2) {
      k[1] = d1[1];
      b[1] = d1[0];
      b[0] = d2[0];
      break;
    }
  }
}

///*Compute a representation k,b for the Sturm chain of n-th degree polynomial p
///(where n is assumed to be >=2).
// the chain will need n+1 entries in both k and b.
// the function should be provided n+1 entries of scratch space in d, in total
// 3n+3 spaces*/
// template <typename T> inline void i_construct_sturm_chain(const T *p, T *k, T
// *b, T* d, int n)
//{
//  T *f, *fmo, *fmt;
//  i_copy(p, d, n+1); /*make a local copy of const pointer p*/
//  f = d;
//  fmo = k;
//  fmt = b;
//  /*compute derivative of polynomial and store results in k*/
//  i_poly_deriv(f, k, n);
//  for (int i = n; i >= 2; i--)
//  {
//    /*f(x)=(k[i]*x+m[i])*fmo(x)-fmt(x), which means k[i]*fmo[i-1]=f[i]*/
//    k[i] = i_div(f[i], fmo[i - 1]);
//    /*and k[i]*fmo[i-2]+m[i]*fmo[i-1]=f[i-1]*/
//    b[i] = i_div(f[i - 1] - k[i] * fmo[i - 2], fmo[i - 1]);
//    /*Compute fmt= -remainder by
//    fmt[j]=k[i]*fmo[j-1]+m[i]*fmo[j]-f[j]*/
//    for (int j = i - 2; j >= 1; j--)
//    {
//      fmt[j] = k[i] * fmo[j - 1] + b[i] * fmo[j] - f[j];
//    }
//    fmt[0] = b[i] * fmo[0] - f[0];
//    /*Shuffle the pointers*/
//    f = fmo;
//    fmo = fmt;
//    fmt = f;
//  }
//  k[1] = f[1];
//  b[1] = f[0];
//  b[0] = fmo[0];
//  //k[0] = 0;
//}

/*Evaluate the Sturm chain and compute the number of sign flips at infinity with
 * sign x (+1 or -1)*/
template <typename T>
inline int i_eval_sturm_chain_inf(const T *k, const T *b, int n, T x) {
  int nr_flips = 0;
  T fi, fim1, fim2;
  fim1 = k[1] * x;
  fim2 = b[0];
  if (fim1 * fim2 <= (T)0.0) {
    nr_flips++;
  }
  for (int i = 2; i <= n; i++) {
    fi = (k[i] * x) * fim1;
    if (fi * fim1 <= (T)0.0) {
      nr_flips++;
    }
    fim2 = fim1;
    fim1 = fi;
  }
  return nr_flips;
}

/*Evaluate the Sturm chain and compute the number of sign flips at the point x*/
template <typename T>
inline int i_eval_sturm_chain(const T *k, const T *b, int n, T x) {
  int nr_flips = 0;
  T fi, fim1, fim2;
  fim1 = k[1] * x + b[1];
  fim2 = b[0];
  if (fim1 * fim2 <= (T)0.0) {
    nr_flips++;
  }
  for (int i = 2; i <= n; i++) {
    fi = (k[i] * x + b[i]) * fim1 - fim2;
    if (fi * fim1 <= (T)0.0) {
      nr_flips++;
    }
    fim2 = fim1;
    fim1 = fi;
  }
  return nr_flips;
}

/*Run max_bisection_iterations number of iterations of bisection on the n-th
degree polynomial p to find a
root in the interval x_min to x_max, assuming that there is a unique root in the
interval*/
template <typename T>
inline T i_poly_bisection(
    const T *p, int n, T x_min, T x_max,
    int max_bisection_iterations = I_DEFAULT_MAX_POLY_BISECTION_ITERATIONS) {
  T val_min, val_middle, x_middle;
  val_min = i_poly_eval(p, n, x_min);
  for (int i = 0; i < max_bisection_iterations; i++) {
    x_middle = i_average(x_min, x_max);
    val_middle = i_poly_eval(p, n, x_middle);
    if (val_middle * val_min > (T)(0.0)) {
      x_min = x_middle;
    } else {
      x_max = x_middle;
    }
  }
  return i_average(x_min, x_max);
}

/*Solve for the real roots of the n-th degree polynomial p using
Sturm-sequences.
return the number of real roots, and place the actual roots at the pointer
'roots'.
the function should be provided 4*n+2 entries of scratch space in mem_d and 2*n
entries in mem_i.*/
template <typename T>
inline int i_poly_solve_sturm(
    const T *p, T *roots, T *mem_d, int *mem_i, int n,
    int max_sturm_iterations = I_DEFAULT_MAX_POLY_STURMCHAIN_ITERATIONS,
    int max_bisec_iterations = I_DEFAULT_MAX_POLY_BISECTION_ITERATIONS) {
  T *k, *b, *d, *stack_d;
  T left_end, right_end, x_middle;
  int *stack_i;
  int flips_neg, flips_pos, flips_left, flips_right, flips_middle;
  int nr_pos, nr_real, flips_zero;
  int nr_iter, nr_roots, nr_stacked, nr_captured;
  /*handle the special case when n<2*/
  if (n <= 1) {
    if (p[1] == (T)(0.0)) /*a constant polynomial*/
    {
      return 0;
    }
    roots[0] = -i_div(p[0], p[1]); /*a linear polynomial*/
    return 1;
  }
  /*assign pointers to the scratch space*/
  k = mem_d;
  b = k + n + 1;
  d = b + n + 1;
  /*compute a representation for the Sturm chain*/
  i_construct_sturm_chain(p, k, b, d, n);
  /*count how many real roots the polynomial has*/
  flips_neg = i_eval_sturm_chain_inf(k, b, n, (T)(-1.0));
  flips_pos = i_eval_sturm_chain_inf(k, b, n, (T)(1.0));
  nr_real = i_min(n, flips_neg - flips_pos);
  /*compute the number of negative and positive roots*/
  flips_zero = i_eval_sturm_chain(k, b, n, (T)(0.0));
  // nr_neg = flips_neg - flips_zero;
  nr_pos = i_min(n, flips_zero - flips_pos);
  /*no roots found yet*/
  nr_roots = 0;
  /*initialize stack*/
  nr_stacked = 0;
  stack_d = d;
  stack_i = mem_i;

  /*search right-open infinite intervals and stack inner intervals as
   * appropriate*/
  for (flips_left = flips_zero, left_end = (T)(0.0), right_end = (T)(1.0),
      nr_iter = 0;
       (nr_roots < nr_pos) && (nr_iter < max_sturm_iterations);) {
    if (flips_left <= flips_pos) {
      break;
    }
    flips_right = i_eval_sturm_chain(k, b, n, right_end);
    nr_captured = flips_left - flips_right;
    if (nr_captured <= 0) {
      left_end = right_end;
      right_end *= (T)(2.0);
      nr_iter++;
    } else {
      /*search the left interval*/
      if (nr_captured == 1) {
        /*one root, do bisection*/
        if (nr_roots < nr_pos) {
          roots[nr_roots++] =
              i_poly_bisection(p, n, left_end, right_end, max_bisec_iterations);
        }
      } else {
        /*has more than one root, stack interval for later*/
        if (nr_stacked < nr_pos) {
          nr_stacked++;
          *stack_i++ = flips_left;
          *stack_i++ = flips_right;
          *stack_d++ = left_end;
          *stack_d++ = right_end;
        } else {
          break;
        }
      }
      /*set up to search the rest of the half-open interval*/
      flips_left = flips_right;
      left_end = right_end;
      right_end *= (T)(2.0);
      nr_iter = 0;
    }
  }

  /*search with left-open infinite intervals and stack inner intervals as
   * appropriate*/
  for (flips_right = flips_zero, right_end = (T)(0.0), left_end = (T)(-1.0),
      nr_iter = 0;
       (nr_roots < nr_real) && (nr_iter < max_sturm_iterations);) {
    if (flips_right >= flips_neg) {
      break;
    }
    flips_left = i_eval_sturm_chain(k, b, n, left_end);
    nr_captured = flips_left - flips_right;
    if (nr_captured <= 0) {
      right_end = left_end;
      left_end *= (T)(2.0);
      nr_iter++;
    } else {
      /*search the right interval*/
      if (nr_captured == 1) {
        /*one root, do bisection*/
        if (nr_roots < nr_real) {
          roots[nr_roots++] =
              i_poly_bisection(p, n, left_end, right_end, max_bisec_iterations);
        }
      } else {
        /*more than one root, stack interval for later*/
        if (nr_stacked < nr_real) {
          nr_stacked++;
          *stack_i++ = flips_left;
          *stack_i++ = flips_right;
          *stack_d++ = left_end;
          *stack_d++ = right_end;
        } else
          break;
      }
      /*set up to search the rest of the half-open interval*/
      flips_right = flips_left;
      right_end = left_end;
      left_end *= (T)(2.0);
      nr_iter = 0;
    }
  }

  /*clear the stack of waiting intervals as appropriate*/
  for (nr_iter = 0; (nr_stacked > 0) && (nr_roots < nr_real) &&
                    (nr_stacked < nr_real) &&
                    (nr_iter < max_sturm_iterations);) {
    /*pop interval from stack*/
    right_end = *(--stack_d);
    left_end = *(--stack_d);
    flips_right = *(--stack_i);
    flips_left = *(--stack_i);
    nr_stacked--;

    /*handle interval*/
    x_middle = i_average(left_end, right_end);
    flips_middle = i_eval_sturm_chain(k, b, n, x_middle);
    /*1st half*/
    nr_captured = flips_left - flips_middle;
    if (nr_captured > 0) {
      /*search the right interval*/
      if (nr_captured == 1) {
        /*one root, do bisection*/
        if (nr_roots < nr_real) {
          roots[nr_roots++] =
              i_poly_bisection(p, n, left_end, x_middle, max_bisec_iterations);
        }
        nr_iter = 0;
      } else {
        /*more than one root, stack interval for later*/
        if (nr_stacked < nr_real) {
          nr_stacked++;
          *stack_i++ = flips_left;
          *stack_i++ = flips_middle;
          *stack_d++ = left_end;
          *stack_d++ = x_middle;
          nr_iter++;
        }
      }
    }
    /*2nd half*/
    nr_captured = flips_middle - flips_right;
    if (nr_captured > 0) {
      /*search the right interval*/
      if (nr_captured == 1) {
        /*one root, do bisection*/
        if (nr_roots < nr_real) {
          roots[nr_roots++] =
              i_poly_bisection(p, n, x_middle, right_end, max_bisec_iterations);
        }
        nr_iter = 0;
      } else {
        /*more than one root, stack interval for later*/
        if (nr_stacked < nr_real) {
          nr_stacked++;
          *stack_i++ = flips_middle;
          *stack_i++ = flips_right;
          *stack_d++ = x_middle;
          *stack_d++ = right_end;
          nr_iter++;
        }
      }
    }
  }
  return nr_roots;
}

/*Solve for the real roots of the tenth degree polynomial p using
 * Sturm-sequences*/
template <typename T>
inline int i_poly_solve_sturm10(
    const T p[11], T roots[10],
    int max_sturm_iterations = I_DEFAULT_MAX_POLY_STURMCHAIN_ITERATIONS,
    int max_bisec_iterations = I_DEFAULT_MAX_POLY_BISECTION_ITERATIONS) {
  T mem_d[42];
  int mem_i[20];
  return i_poly_solve_sturm(p, roots, mem_d, mem_i, 10, max_sturm_iterations,
                            max_bisec_iterations);
}

/*Closed form solution for solving quadratic polynomial equation
  p[2]*x^2 + p[1]*x + p[0] = 0.
  Return number of real roots and output actual real roots in roots (up to 2
  real roots)*/
template <typename T>
inline int i_quadratic_solve_closed(const T p[3], T roots[2]) {
  int nr_roots = 0;
  i_zero2(roots);
  if (p[2] == (T)0.0) {
    if (p[1] == (T)0.0) {
      return 0;
    }
    roots[0] = -i_div(p[0], p[1]);
    return 1;
  }
  T mp1 = -p[1];
  T rec_2p2 = i_rec(p[2]) * (T)(0.5);
  T discriminant = i_sqr(p[1]) - 4 * p[2] * p[0];

  if (discriminant < (T)0.0) {
    return 0;
  }
  T sqrt_discriminant = i_sqrt(discriminant);
  if (discriminant == (T)0.0) {
    nr_roots = 1;
    roots[0] = roots[1] = (mp1 * rec_2p2);
  } else {
    nr_roots = 2;
    roots[0] = (mp1 + sqrt_discriminant) * rec_2p2;
    roots[1] = (mp1 - sqrt_discriminant) * rec_2p2;
  }
  return nr_roots;
}

/*Closed form solution for solving cubic polynomial equation
  p[3]*x^3 + p[2]*x^2 + p[1]*x + p[0] = 0.
  Return actual number of real roots and output real roots in roots (up to 3
  real roots)
  if "force_three_roots" is true, always try for three roots*/
template <typename T>
inline int i_cubic_solve_closed(const T p[4], T roots[3],
                                bool force_three_roots = false) {
  int nr_roots = 0; /*number of actual roots*/
  i_zero3(roots);
  if (p[3] == (T)0.0) /*a quadratic equation, if force_three_roots, the rest
                         roots are filled with zeros*/
  {
    return i_quadratic_solve_closed(p, roots);
  }
  T theta = (T)0.0, sqrtqm2, mbo3, thetao3, sqrtq, ay, mqoay;
  /*normalize coefficients such that p[3] = 1.0*/
  T rec_p3 = i_rec(p[3]);
  T b = p[2] * rec_p3;
  T c = p[1] * rec_p3;
  T d = p[0] * rec_p3;
  T b_sqr = i_sqr(b);
  /*define the intermediate variables q and r*/
  T q = (3 * c - b_sqr) / (T)9.0;
  T r = (9 * b * c - 27 * d - 2 * b_sqr * b) / (T)54.0;
  T q3 = i_cub(q);
  T r2 = i_sqr(r);
  T discriminant = r2 + q3;
  if (discriminant <= (T)0) /*three actual roots*/
  {
    sqrtq = i_sqrt(
        -q); /*(r2 = r*r) >= 0. r2 + q3 < 0 so q3 must be <= 0, so q must <= 0*/
    theta = i_acos(i_div(r, i_sqrt(-q3)));
    sqrtqm2 = (T)2.0 * sqrtq;
    mbo3 = -b / (T)3.0;
    thetao3 = theta / (T)(3.0);
    roots[0] = sqrtqm2 * i_cos(thetao3) + mbo3;
    roots[1] = sqrtqm2 * i_cos(thetao3 + Constant<T>::TWO_PI() / (T)3.0) + mbo3;
    roots[2] = sqrtqm2 * i_cos(thetao3 - Constant<T>::TWO_PI() / (T)3.0) + mbo3;
    nr_roots = 3;
  } else {
    nr_roots = 1; /*single root*/
    if (force_three_roots) {
      /*try for 3 roots anyway*/
      /*(r2 = r*r) >= 0. r2 + q3 > 0 so first make sure q is positive or
       * negative*/
      if (q < (T)0) {
        sqrtq = i_sqrt(-q);
        theta = i_acos(i_div(r, i_sqrt(-q3)));
        sqrtqm2 = (T)2.0 * sqrtq;
        mbo3 = -b / (T)3.0;
        thetao3 = theta / (T)(3.0);
        roots[0] = sqrtqm2 * i_cos(thetao3) + mbo3;
        roots[1] =
            sqrtqm2 * i_cos(thetao3 + Constant<T>::TWO_PI() / (T)3.0) + mbo3;
        roots[2] =
            sqrtqm2 * i_cos(thetao3 - Constant<T>::TWO_PI() / (T)3.0) + mbo3;
      } else {
        mbo3 = -b / (T)3.0;
        i_fill3(roots, mbo3);
      }
    } else /*do not try for 3 roots, just return the actual single root*/
    {
      ay = i_sign(r) * i_cbrt(i_abs(r) + i_sqrt(discriminant));
      mqoay = (ay != (T)0.0) ? i_div(-q, ay) : (T)0.0;
      roots[0] = ay + mqoay - b / (T)3.0;
    }
  }
  return nr_roots;
}

/*Closed form solution for solving cubic polynomial equation
 p[4]*x^4+p[3]*x^3+p[2]*x^2+p[1]*x^1+p[0].
 Return actual number of real roots and output real roots in roots (up to 3 real
 roots)
 if "force_four_roots" is true, always try for four roots*/
template <typename T>
inline int i_quartic_solve_closed(const T p[5], T roots[4],
                                  bool force_four_roots = false) {
  i_zero4(roots);
  if (p[4] == (T)0.0) {
    /*a cubic equation, if force_four_roots, the rest roots are filled with
     * zeros*/
    return i_cubic_solve_closed(p, roots, force_four_roots);
  }
  /*normalize the first coefficient*/
  T rec_p4 = i_rec(p[4]);
  T c3 = p[3] * rec_p4;
  T c2 = p[2] * rec_p4;
  T c1 = p[1] * rec_p4;
  T c0 = p[0] * rec_p4;
  T c3_half = c3 * (T)0.5;
  T c3_half_sqr = i_sqr(c3_half);
  T c2_m_c3_half_sqr = c2 - c3_half_sqr;
  T c1_quarter = c1 * (T)0.25;
  T k[4], crt[3], p1[3], p2[3], qrt1[2], qrt2[2];
  k[0] = c2_m_c3_half_sqr * c0 - c1_quarter * c1;
  k[1] = -c0 + c1_quarter * c3;
  k[2] = -(c2_m_c3_half_sqr + c3_half_sqr) * (T)0.25;
  k[3] = (T)0.25;
  if (0 == i_cubic_solve_closed(k, crt)) {
    return 0;
  }
  T crt0 = crt[0];
  T m2 = crt0 - c2_m_c3_half_sqr;
  T n2 = i_sqr(crt0) * (T)0.25 - c0;
  T mn = crt0 * c3 * (T)0.5 - c1;
  if (m2 < 0 || n2 < 0) {
    if (!force_four_roots) {
      return (0);
    } else {
      if (m2 < 0) {
        m2 = (T)0.0;
      }
      if (n2 < 0) {
        n2 = (T)0.0;
      }
    }
  }
  T m = i_sqrt(m2);
  T n = i_sqrt(n2) * i_sign_never_zero(mn);
  T crt0_half = crt0 * (T)0.5;
  p1[0] = crt0_half + n;
  p1[1] = c3_half + m;
  p1[2] = (T)1.0;
  p2[0] = crt0_half - n;
  p2[1] = c3_half - m;
  p2[2] = (T)1.0;
  int nr_roots_p1 = i_quadratic_solve_closed(p1, qrt1);
  int nr_roots_p2 = i_quadratic_solve_closed(p2, qrt2);
  if (nr_roots_p1 == 0) {
    i_copy2(qrt2, roots);
  } else if (nr_roots_p1 == 1) {
    if (nr_roots_p2 == 0) {
      i_copy2(qrt1, roots);
    } else if (nr_roots_p2 == 1) {
      roots[0] = roots[2] = qrt1[0];
      roots[1] = roots[3] = qrt2[0];
    } else {
      roots[0] = roots[3] = qrt1[0];
      i_copy2(qrt2, roots + 1);
    }
  } else {
    i_copy2(qrt1, roots);
    i_copy2(qrt2, roots + 2);
  }
  return (nr_roots_p1 + nr_roots_p2);
}
} /* namespace idl */