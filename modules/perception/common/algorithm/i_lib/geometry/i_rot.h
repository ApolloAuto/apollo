#pragma once

#include "modules/perception/common/algorithm/core/i_blas.h"
#include "modules/perception/common/algorithm/core/i_rand.h"
// todo(daohu527): need complete?
#include "modules/perception/common/algorithm/numeric/i_eig.h"
#include "modules/perception/common/algorithm/numeric/i_svd.h"

namespace idl {
/*This routine force a 2x2 orthogonal matrix R in place (i.e., force it to be
rotation maxtrix with positive determinant),
/*by unitizing the first row, and taking its cross as the second*/
template <typename T>
inline void i_force_rot_2x2(T R[4]) {
  i_unitize2(R);
  R[2] = -R[1];
  R[3] = R[0];
}

/*This routine orthogonalizes a 3x3 near Rotation matrix R in place, i.e., force
 * it to be orthogonal.*/
template <typename T>
inline void i_rot_orthogonalize(
    T R[9], int nr_svd_iter = I_DEFAULT_MAX_SVD_ITERATIONS) {
  /*obtain SVD*/
  T Ut[9], w[3], Vt[9];
  i_svd_3x3(R, Ut, w, Vt, true, nr_svd_iter);
  if (i_determinant_3x3(Ut) < (T)0.0) {
    i_neg9(Ut);
  }
  if (i_determinant_3x3(Vt) < (T)0.0) {
    i_neg9(Vt);
  }
  i_mult_AtB_3x3_3x3(Ut, Vt, R);
}

/*Rodrigues' solver for computing the matrix exponential of a 3x3 skew-symmetric
 * matrix*/
template <typename T>
inline void i_rot_rodrigues_3x3_solver(T sinc, T mcos, T a0_sqr, T a1_sqr,
                                       T a2_sqr, const T a[3], T R[9]) {
  T tmp1, tmp2;
  R[0] = (T)1.0 - (a1_sqr + a2_sqr) * mcos;
  R[4] = (T)1.0 - (a0_sqr + a2_sqr) * mcos;
  R[8] = (T)1.0 - (a0_sqr + a1_sqr) * mcos;
  tmp1 = a[0] * a[1] * mcos;
  tmp2 = a[2] * sinc;
  R[1] = tmp1 - tmp2;
  R[3] = tmp1 + tmp2;
  tmp1 = a[2] * a[0] * mcos;
  tmp2 = a[1] * sinc;
  R[2] = tmp1 + tmp2;
  R[6] = tmp1 - tmp2;
  tmp1 = a[1] * a[2] * mcos;
  tmp2 = a[0] * sinc;
  R[5] = tmp1 - tmp2;
  R[7] = tmp1 + tmp2;
}

template <typename T>
inline void i_rot_rodrigues_3x3_solver(T sinc, T mcos, T a0_sqr, T a1_sqr,
                                       T a2_sqr, const T s_da[3],
                                       const T c_da[3], const T a[3], T R[9],
                                       T D[9][3]) {
  T tmp1, tmp2, e_x[9], e_x2[9];
  T mcos_x_a0 = mcos * a[0];
  T mcos_x_a1 = mcos * a[1];
  T mcos_x_a2 = mcos * a[2];

  T mtwo_mcos_x_a0 = -(2 * mcos_x_a0);
  T mtwo_mcos_x_a1 = -(2 * mcos_x_a1);
  T mtwo_mcos_x_a2 = -(2 * mcos_x_a2);

  R[0] = (T)1.0 - (a1_sqr + a2_sqr) * mcos;
  R[4] = (T)1.0 - (a0_sqr + a2_sqr) * mcos;
  R[8] = (T)1.0 - (a0_sqr + a1_sqr) * mcos;

  tmp1 = a[0] * mcos_x_a1;
  tmp2 = a[2] * sinc;
  R[1] = tmp1 - tmp2;
  R[3] = tmp1 + tmp2;
  tmp1 = a[2] * mcos_x_a0;
  tmp2 = a[1] * sinc;
  R[2] = tmp1 + tmp2;
  R[6] = tmp1 - tmp2;
  tmp1 = a[1] * mcos_x_a2;
  tmp2 = a[0] * sinc;
  R[5] = tmp1 - tmp2;
  R[7] = tmp1 + tmp2;

  /*derivatives*/
  i_axiator(a, e_x);
  i_sqr_skew_symmetric_3x3(a, e_x2);

  // i=0
  D[0][0] = (T)(/*e_x[0] * s_da[0]*/ /*+ sinc*e_x_da[0][0] +*/ e_x2[0] *
                c_da[0] /*+ mcos * e_x2_da[0][0]*/);
  D[0][1] =
      (T)(/*e_x[0] * s_da[1]*/ /*+ sinc*e_x_da[0][1] +*/ e_x2[0] * c_da[1] +
          mtwo_mcos_x_a1);
  D[0][2] =
      (T)(/*e_x[0] * s_da[2]*/ /*+ sinc*e_x_da[0][2] +*/ e_x2[0] * c_da[2] +
          mtwo_mcos_x_a2);
  // i=1
  D[1][0] = (T)(e_x[1] * s_da[0] /*+ sinc*e_x_da[1][0]*/ + e_x2[1] * c_da[0] +
                mcos_x_a1);
  D[1][1] = (T)(e_x[1] * s_da[1] /*+ sinc*e_x_da[1][1]*/ + e_x2[1] * c_da[1] +
                mcos_x_a0);
  D[1][2] = (T)(e_x[1] * s_da[2] - sinc /**e_x_da[1][2]*/ +
                e_x2[1] * c_da[2] /*+ mcos * e_x2_da[1][2]*/);
  // i=2
  D[2][0] = (T)(e_x[2] * s_da[0] /*+ sinc*e_x_da[2][0]*/ + e_x2[2] * c_da[0] +
                mcos_x_a2);
  D[2][1] = (T)(e_x[2] * s_da[1] + sinc /**e_x_da[2][1]*/ +
                e_x2[2] * c_da[1] /*+ mcos * e_x2_da[2][1]*/);
  D[2][2] = (T)(e_x[2] * s_da[2] /*+ sinc*e_x_da[2][2]*/ + e_x2[2] * c_da[2] +
                mcos_x_a0);
  // i=3
  D[3][0] = (T)(e_x[3] * s_da[0] /*+ sinc*e_x_da[3][0]*/ + e_x2[3] * c_da[0] +
                mcos_x_a1);
  D[3][1] = (T)(e_x[3] * s_da[1] /*+ sinc*e_x_da[3][1]*/ + e_x2[3] * c_da[1] +
                mcos_x_a0);
  D[3][2] = (T)(e_x[3] * s_da[2] + sinc /**e_x_da[3][2]*/ +
                e_x2[3] * c_da[2] /*+ mcos * e_x2_da[3][2]*/);
  // i=4
  D[4][0] =
      (T)(/*e_x[4] * s_da[0]*/ /*+ sinc*e_x_da[4][0] +*/ e_x2[4] * c_da[0] +
          mtwo_mcos_x_a0);
  D[4][1] = (T)(/*e_x[4] * s_da[1]*/ /*+ sinc*e_x_da[4][1] +*/ e_x2[4] *
                c_da[1] /*+ mcos * e_x2_da[4][1]*/);
  D[4][2] =
      (T)(/*e_x[4] * s_da[2]*/ /*+ sinc*e_x_da[4][2] +*/ e_x2[4] * c_da[2] +
          mtwo_mcos_x_a2);
  // i=5
  D[5][0] = (T)(e_x[5] * s_da[0] - sinc /**e_x_da[5][0]*/ +
                e_x2[5] * c_da[0] /*+ mcos * e_x2_da[5][0]*/);
  D[5][1] = (T)(e_x[5] * s_da[1] /*+ sinc*e_x_da[5][1]*/ + e_x2[5] * c_da[1] +
                mcos_x_a2);
  D[5][2] = (T)(e_x[5] * s_da[2] /*+ sinc*e_x_da[5][2]*/ + e_x2[5] * c_da[2] +
                mcos_x_a1);
  // i=6
  D[6][0] = (T)(e_x[6] * s_da[0] /*+ sinc*e_x_da[6][0]*/ + e_x2[6] * c_da[0] +
                mcos_x_a2);
  D[6][1] = (T)(e_x[6] * s_da[1] - sinc /**e_x_da[6][1]*/ +
                e_x2[6] * c_da[1] /*+ mcos * e_x2_da[6][1]*/);
  D[6][2] = (T)(e_x[6] * s_da[2] /*+ sinc*e_x_da[6][2]*/ + e_x2[6] * c_da[2] +
                mcos_x_a0);
  // i=7
  D[7][0] = (T)(e_x[7] * s_da[0] + sinc /**e_x_da[7][0]*/ +
                e_x2[7] * c_da[0] /*+ mcos * e_x2_da[7][0]*/);
  D[7][1] = (T)(e_x[7] * s_da[1] /*+ sinc*e_x_da[7][1]*/ + e_x2[7] * c_da[1] +
                mcos_x_a2);
  D[7][2] = (T)(e_x[7] * s_da[2] /*+ sinc*e_x_da[7][2]*/ + e_x2[7] * c_da[2] +
                mcos_x_a1);
  // i=8
  D[8][0] =
      (T)(/*e_x[8] * s_da[0]*/ /*+ sinc*e_x_da[8][0] +*/ e_x2[8] * c_da[0] +
          mtwo_mcos_x_a0);
  D[8][1] =
      (T)(/*e_x[8] * s_da[1]*/ /*+ sinc*e_x_da[8][1] +*/ e_x2[8] * c_da[1] +
          mtwo_mcos_x_a1);
  D[8][2] = (T)(/*e_x[8] * s_da[2]*/ /*+ sinc*e_x_da[8][2] +*/ e_x2[8] *
                c_da[2] /*+ mcos * e_x2_da[8][2]*/);
}

/*Rodrigues' formula for computing the matrix exponential of a 3x3
  skew-symmetric matrix The 1st argument is the input (a 3-vector) and the 2nd
  argument is the output (a 3x3 rotation matrix)*/
template <typename T>
inline void i_rot_rodrigues_3x3(const T a[3], T R[9]) {
  T sinc, mcos, x[3];
  i_copy3(a, x);
  T a0_sqr = i_sqr(x[0]);
  T a1_sqr = i_sqr(x[1]);
  T a2_sqr = i_sqr(x[2]);
  T theta2 = a0_sqr + a1_sqr + a2_sqr;
  T theta = i_sqrt(theta2);

  if (theta < Constant<T>::MIN_ABS_SAFE_DIVIDEND()) {
    /*theta is too small; use a third-order Taylor approximation for sin(theta)
     * and cos(theta)*/
    sinc = (T)1.0 - theta2 / (T)6.0;
    mcos = (T)0.5 - theta2 / (T)24.0;
  } else if (theta > Constant<T>::PI()) {
    /*if ||a||>PI, the it maybe replaced by a(1-2PI/||a||), which represenrs the
     * same rotation*/
    theta = (T)(1.0) - Constant<T>::TWO_PI() / theta;
    i_scale3(x, theta);
    a0_sqr = i_sqr(x[0]);
    a1_sqr = i_sqr(x[1]);
    a2_sqr = i_sqr(x[2]);
    theta2 = a0_sqr + a1_sqr + a2_sqr;
    theta = i_sqrt(theta2);
    if (theta < Constant<T>::MIN_ABS_SAFE_DIVIDEND()) {
      /*theta is too small; use a third-order Taylor approximation for
       * sin(theta) and cos(theta)*/
      sinc = (T)1.0 - theta2 / (T)6.0;
      mcos = (T)0.5 - theta2 / (T)24.0;
    } else {
      sinc = i_div(i_sin(theta), theta);
      mcos = i_div((T)1.0 - i_cos(theta), theta2);
    }
  } else {
    sinc = i_div(i_sin(theta), theta);
    mcos = i_div((T)1.0 - i_cos(theta), theta2);
  }
  i_rot_rodrigues_3x3_solver(sinc, mcos, a0_sqr, a1_sqr, a2_sqr, x, R);
}

/*Rodrigues' formula for computing the matrix exponential of a 3x3
skew-symmetric matrix The 1st argument is the input (a 3-vector), the 2nd
argument is the output (a 3x3 rotation matrix), The 3rd argument is the
derivative (a 9x3 matrix)*/
template <typename T>
inline void i_rot_rodrigues_3x3(const T a[3], T R[9], T D[9][3]) {
  T sinc, mcos, x[3], s_da[3],
      c_da[3]; /*derivatives of sinc and mcos wrt to a, respectively*/
  i_copy3(a, x);

  T a0_sqr = i_sqr(x[0]);
  T a1_sqr = i_sqr(x[1]);
  T a2_sqr = i_sqr(x[2]);

  T theta2 = a0_sqr + a1_sqr + a2_sqr;
  T theta = i_sqrt(theta2);

  if (theta < Constant<T>::MIN_ABS_SAFE_DIVIDEND()) {
    /*theta is too small; use a third-order Taylor approximation for sin(theta)
     * and cos(theta)*/
    sinc = (T)1.0 - theta2 / (T)6.0;
    mcos = (T)0.5 - theta2 / (T)24.0;
    /*derivatives*/
    i_scale3(x, s_da, -i_rec((T)3.0));
    i_scale3(x, c_da, -i_rec((T)12.0));
  } else if (theta > Constant<T>::PI()) {
    /*if ||a||>PI, the it maybe replaced by a(1-2PI/||a||), which represenrs the
     * same rotation*/
    theta = (T)(1.0) - Constant<T>::TWO_PI() / theta;
    i_scale3(x, theta);
    a0_sqr = i_sqr(x[0]);
    a1_sqr = i_sqr(x[1]);
    a2_sqr = i_sqr(x[2]);
    theta2 = a0_sqr + a1_sqr + a2_sqr;
    theta = i_sqrt(theta2);
    if (theta < Constant<T>::MIN_ABS_SAFE_DIVIDEND()) {
      /*theta is too small; use a third-order Taylor approximation for
       * sin(theta) and cos(theta)*/
      sinc = (T)1.0 - theta2 / (T)6.0;
      mcos = (T)0.5 - theta2 / (T)24.0;
      /*derivatives*/
      i_scale3(x, s_da, -i_rec((T)3.0));
      i_scale3(x, c_da, -i_rec((T)12.0));
    } else {
      T sin_theta = i_sin(theta);
      T cos_theta = i_cos(theta);
      sinc = i_div(sin_theta, theta);
      mcos = i_div((T)1.0 - cos_theta, theta2);
      /*derivatives*/
      T s_dtheta = (cos_theta - sin_theta / theta) / theta2;
      T c_dtheta = (sinc - (T)(2.0) * mcos) / theta2;
      i_scale3(x, s_da, s_dtheta);
      i_scale3(x, c_da, c_dtheta);
    }
  } else {
    T sin_theta = i_sin(theta);
    T cos_theta = i_cos(theta);
    sinc = i_div(sin_theta, theta);
    mcos = i_div((T)1.0 - cos_theta, theta2);
    /*derivatives*/
    T s_dtheta = (cos_theta - sin_theta / theta) / theta2;
    T c_dtheta = (sinc - (T)(2.0) * mcos) / theta2;
    i_scale3(x, s_da, s_dtheta);
    i_scale3(x, c_da, c_dtheta);
  }
  i_rot_rodrigues_3x3_solver(sinc, mcos, a0_sqr, a1_sqr, a2_sqr, s_da, c_da, x,
                             R, D);
}

/*Rodrigues' formula for computing the matrix exponential of a 3x3
skew-symmetric matrix The 1st argument is the input (a 3-vector), the 2nd
argument is the output (a 3x3 rotation matrix), The 3rd argument is the
derivative (a 9x3 matrix) Keep this slow implementation as baseline - Liang
July/31/2015*/
template <typename T>
inline void i_rot_rodrigues_3x3_slow(const T a[3], T R[9], T D[9][3]) {
  T sinc, mcos, tmp1, tmp2;
  T a0_sqr = i_sqr(a[0]);
  T a1_sqr = i_sqr(a[1]);
  T a2_sqr = i_sqr(a[2]);

  T theta2 = a0_sqr + a1_sqr + a2_sqr;
  T theta = i_sqrt(theta2);
  T s_da[3], c_da[3]; /*derivatives of sinc and mcos wrt to a, respectively*/

  if (theta < Constant<T>::MIN_ABS_SAFE_DIVIDEND()) {
    /*theta is too small; use a third-order Taylor approximation for sin(theta)
     * and cos(theta)*/
    sinc = (T)1.0 - theta2 / (T)6.0;
    mcos = (T)0.5 - theta2 / (T)24.0;
    /*derivatives*/
    i_scale3(a, s_da, -i_rec((T)3.0));
    i_scale3(a, c_da, -i_rec((T)12.0));
    // s_da[0] = -a[0]/(T)3.0; c_da[0] = -a[0]/12.0;
    // s_da[1] = -a[1]/(T)3.0; c_da[1] = -a[1]/12.0;
    // s_da[2] = -a[2]/(T)3.0; c_da[2] = -a[2]/12.0;
  } else {
    T sin_theta = i_sin(theta);
    T cos_theta = i_cos(theta);
    sinc = i_div(sin_theta, theta);
    mcos = i_div((T)1.0 - cos_theta, theta2);
    /*derivatives*/
    T s_dtheta = (cos_theta - sin_theta / theta) / theta2;
    T c_dtheta = (sinc - (T)(2.0) * mcos) / theta2;
    i_scale3(a, s_da, s_dtheta);
    i_scale3(a, c_da, c_dtheta);
    // s_da[0] = s_dtheta*a[0]; c_da[0] = c_dtheta*a[0];
    // s_da[1] = s_dtheta*a[1]; c_da[1] = c_dtheta*a[1];
    // s_da[2] = s_dtheta*a[2]; c_da[2] = c_dtheta*a[2];
  }
  R[0] = (T)1.0 - (a1_sqr + a2_sqr) * mcos;
  R[4] = (T)1.0 - (a0_sqr + a2_sqr) * mcos;
  R[8] = (T)1.0 - (a0_sqr + a1_sqr) * mcos;
  tmp1 = a[0] * a[1] * mcos;
  tmp2 = a[2] * sinc;
  R[1] = tmp1 - tmp2;
  R[3] = tmp1 + tmp2;
  tmp1 = a[2] * a[0] * mcos;
  tmp2 = a[1] * sinc;
  R[2] = tmp1 + tmp2;
  R[6] = tmp1 - tmp2;
  tmp1 = a[1] * a[2] * mcos;
  tmp2 = a[0] * sinc;
  R[5] = tmp1 - tmp2;
  R[7] = tmp1 + tmp2;

  /*derivatives*/
  T e_x[9];
  i_axiator(a, e_x);

  static const T e_x_da[9][3] = {
      {(T)0, (T)0, (T)0},  {(T)0, (T)0, -(T)1}, {(T)0, (T)1, (T)0},
      {(T)0, (T)0, (T)1},  {(T)0, (T)0, (T)0},  {-(T)1, (T)0, (T)0},
      {(T)0, -(T)1, (T)0}, {(T)1, (T)0, (T)0},  {(T)0, (T)0, (T)0}};

  T e_x2[9];
  i_sqr_skew_symmetric_3x3(a, e_x2);

  //\frac{d {\hat v}^2}{d v}
  T e_x2_da[9][3];
  e_x2_da[0][0] = e_x2_da[1][2] = e_x2_da[2][1] = (T)0.0;
  e_x2_da[3][2] = e_x2_da[4][1] = e_x2_da[5][0] = (T)0.0;
  e_x2_da[6][1] = e_x2_da[7][0] = e_x2_da[8][2] = (T)0.0;
  e_x2_da[1][1] = e_x2_da[3][1] = e_x2_da[2][2] = e_x2_da[6][2] = a[0];
  e_x2_da[1][0] = e_x2_da[3][0] = e_x2_da[5][2] = e_x2_da[7][2] = a[1];
  e_x2_da[2][0] = e_x2_da[6][0] = e_x2_da[5][1] = e_x2_da[7][1] = a[2];
  e_x2_da[4][0] = e_x2_da[8][0] = -(T)2.0 * a[0];
  e_x2_da[0][1] = e_x2_da[8][1] = -(T)2.0 * a[1];
  e_x2_da[0][2] = e_x2_da[4][2] = -(T)2.0 * a[2];

  /*for (int i = 0; i<9; ++i)
  {
  D[i][0] = (T)(e_x[i] * s_da[0] + sinc*e_x_da[i][0] + e_x2[i] * c_da[0] + mcos
  * e_x2_da[i][0]); D[i][1] = (T)(e_x[i] * s_da[1] + sinc*e_x_da[i][1] + e_x2[i]
  * c_da[1] + mcos * e_x2_da[i][1]); D[i][2] = (T)(e_x[i] * s_da[2] +
  sinc*e_x_da[i][2] + e_x2[i] * c_da[2] + mcos * e_x2_da[i][2]);
  }*/

  // i=0
  D[0][0] = (T)(e_x[0] * s_da[0] + sinc * e_x_da[0][0] + e_x2[0] * c_da[0] +
                mcos * e_x2_da[0][0]);
  D[0][1] = (T)(e_x[0] * s_da[1] + sinc * e_x_da[0][1] + e_x2[0] * c_da[1] +
                mcos * e_x2_da[0][1]);
  D[0][2] = (T)(e_x[0] * s_da[2] + sinc * e_x_da[0][2] + e_x2[0] * c_da[2] +
                mcos * e_x2_da[0][2]);
  // i=1
  D[1][0] = (T)(e_x[1] * s_da[0] + sinc * e_x_da[1][0] + e_x2[1] * c_da[0] +
                mcos * e_x2_da[1][0]);
  D[1][1] = (T)(e_x[1] * s_da[1] + sinc * e_x_da[1][1] + e_x2[1] * c_da[1] +
                mcos * e_x2_da[1][1]);
  D[1][2] = (T)(e_x[1] * s_da[2] + sinc * e_x_da[1][2] + e_x2[1] * c_da[2] +
                mcos * e_x2_da[1][2]);
  // i=2
  D[2][0] = (T)(e_x[2] * s_da[0] + sinc * e_x_da[2][0] + e_x2[2] * c_da[0] +
                mcos * e_x2_da[2][0]);
  D[2][1] = (T)(e_x[2] * s_da[1] + sinc * e_x_da[2][1] + e_x2[2] * c_da[1] +
                mcos * e_x2_da[2][1]);
  D[2][2] = (T)(e_x[2] * s_da[2] + sinc * e_x_da[2][2] + e_x2[2] * c_da[2] +
                mcos * e_x2_da[2][2]);
  // i=3
  D[3][0] = (T)(e_x[3] * s_da[0] + sinc * e_x_da[3][0] + e_x2[3] * c_da[0] +
                mcos * e_x2_da[3][0]);
  D[3][1] = (T)(e_x[3] * s_da[1] + sinc * e_x_da[3][1] + e_x2[3] * c_da[1] +
                mcos * e_x2_da[3][1]);
  D[3][2] = (T)(e_x[3] * s_da[2] + sinc * e_x_da[3][2] + e_x2[3] * c_da[2] +
                mcos * e_x2_da[3][2]);
  // i=4
  D[4][0] = (T)(e_x[4] * s_da[0] + sinc * e_x_da[4][0] + e_x2[4] * c_da[0] +
                mcos * e_x2_da[4][0]);
  D[4][1] = (T)(e_x[4] * s_da[1] + sinc * e_x_da[4][1] + e_x2[4] * c_da[1] +
                mcos * e_x2_da[4][1]);
  D[4][2] = (T)(e_x[4] * s_da[2] + sinc * e_x_da[4][2] + e_x2[4] * c_da[2] +
                mcos * e_x2_da[4][2]);
  // i=5
  D[5][0] = (T)(e_x[5] * s_da[0] + sinc * e_x_da[5][0] + e_x2[5] * c_da[0] +
                mcos * e_x2_da[5][0]);
  D[5][1] = (T)(e_x[5] * s_da[1] + sinc * e_x_da[5][1] + e_x2[5] * c_da[1] +
                mcos * e_x2_da[5][1]);
  D[5][2] = (T)(e_x[5] * s_da[2] + sinc * e_x_da[5][2] + e_x2[5] * c_da[2] +
                mcos * e_x2_da[5][2]);
  // i=6
  D[6][0] = (T)(e_x[6] * s_da[0] + sinc * e_x_da[6][0] + e_x2[6] * c_da[0] +
                mcos * e_x2_da[6][0]);
  D[6][1] = (T)(e_x[6] * s_da[1] + sinc * e_x_da[6][1] + e_x2[6] * c_da[1] +
                mcos * e_x2_da[6][1]);
  D[6][2] = (T)(e_x[6] * s_da[2] + sinc * e_x_da[6][2] + e_x2[6] * c_da[2] +
                mcos * e_x2_da[6][2]);
  // i=7
  D[7][0] = (T)(e_x[7] * s_da[0] + sinc * e_x_da[7][0] + e_x2[7] * c_da[0] +
                mcos * e_x2_da[7][0]);
  D[7][1] = (T)(e_x[7] * s_da[1] + sinc * e_x_da[7][1] + e_x2[7] * c_da[1] +
                mcos * e_x2_da[7][1]);
  D[7][2] = (T)(e_x[7] * s_da[2] + sinc * e_x_da[7][2] + e_x2[7] * c_da[2] +
                mcos * e_x2_da[7][2]);
  // i=8
  D[8][0] = (T)(e_x[8] * s_da[0] + sinc * e_x_da[8][0] + e_x2[8] * c_da[0] +
                mcos * e_x2_da[8][0]);
  D[8][1] = (T)(e_x[8] * s_da[1] + sinc * e_x_da[8][1] + e_x2[8] * c_da[1] +
                mcos * e_x2_da[8][1]);
  D[8][2] = (T)(e_x[8] * s_da[2] + sinc * e_x_da[8][2] + e_x2[8] * c_da[2] +
                mcos * e_x2_da[8][2]);
}

/*The inverse Rodrigues' formula for computing the logarithm of a 3x3 rotation
  matrix The 1st argument is the input (a 3x3 rotation matrix) and the 2nd
  argument is the output (a 3-vector rotation vector)*/
template <typename T>
inline void i_rot_invert_rodrigues_3x3(const T R[9],
                                       T v[3] /*unnormalized rotation vector*/,
                                       T &theta /*rotation angle in radians*/) {
  T r[3];
  T Q[9], Ac[9], h[3], htA[3], ss[3];
  int iv[3];
  const T *R_ref[3];
  T *Q_ref[3], *Ac_ref[3];
  i_make_const_reference_3x3(R, R_ref);
  i_make_reference_3x3(Q, Q_ref);
  i_make_reference_3x3(Ac, Ac_ref);
  /*compute the eigenvector of R corresponding to the eigenvalue 1, every
   * rotation matrix must have this eigenvalue*/
  i_eigenvector_from_eigenvalue(R_ref, (T)(1.0), Q_ref, h, htA, ss, Ac_ref, 3,
                                iv);
  i_copy3(Q + 6, v); /*eigen vector of R, unit rotation axis*/
  r[0] = R[7] - R[5];
  r[1] = R[2] - R[6];
  r[2] = R[3] - R[1];
  T c = (i_trace_3x3(R) - (T)1.0) *
        (T)0.5; /*the trace of R is 1 + 2\cos(\theta), equivalent to the sum of
                   its eigenvalues*/
  T s = i_dot3(r, v) * (T)0.5;
  theta = i_atan2(s, c);
  i_scale3(v, theta);
}

/*This routine finds a rotation matrix R that rotates unit vector an onto unit
  vector b.
  a and b are both assumed to be unit vectors. Corner cases are a and b are
  parallel*/
template <typename T>
inline bool i_rot_3x3(const T a[3], const T b[3], T R[9]) {
  T v[3], s, theta;  // vx[9], vx2[9];
  i_eye_3x3(R);
  /*corner case 1: a == b*/
  if (i_equal3(a, b)) {
    return true;
  }
  i_cross(a, b, v);

  /*L2 norm of v:*/
  s = i_l2_norm3(v); /*sine of angle*/

  if (s < Constant<T>::MIN_ABS_SAFE_DIVIDEND()) {
    return false; /*parallel vectors*/
  }

  /*normalize v:*/
  i_scale3(v, i_rec(s));

  // i_axiator(v, vx);
  // i_mult_AAt_3x3(vx, vx2);
  // i_sqr_skew_symmetric_3x3(vx, vx2);

  // c = i_dot3(a, b);
  theta = i_acos(i_dot3(a, b)); /*dot product - cos(theta)*/

  i_scale3(v, theta);

  i_rot_rodrigues_3x3(v, R);

  // s = i_sin(theta);
  // c = (T)1.0 - c;

  // i_scale9(vx,  s);
  // i_scale9(vx2, c);

  ///*rotation matrix using exponential map*/
  // i_add9(vx, vx2, R);

  // R[0] += (T)1.0;
  // R[4] += (T)1.0;
  // R[8] += (T)1.0;
  //
  // if (i_determinant_3x3(R) < (T)0.0)
  //{
  //  i_neg9(R);
  //}
  return true;
}

/*Generate a random 3x3 rotation matrix R, if force_proper is set to 1, then R
 * is forced to be a proper rotation matrix with det(R) = 1*/
template <typename T>
inline void i_rand_rot_3x3(T R[9], int &seed, int force_proper = 1) {
  T Q[9], A[9], h[3], htA[3], ss[3];
  T *R_ref[3], *Q_ref[3], *A_ref[3];
  int iv[3];
  i_make_reference_3x3(R, R_ref);
  i_make_reference_3x3(Q, Q_ref);
  i_make_reference_3x3(A, A_ref);
  i_rand_orthogonal<T>(Q_ref, A_ref, R_ref, h, htA, ss, 3, 3, iv, seed);
  if (force_proper) {
    if (i_determinant_3x3(R) < (T)0.0) {
      i_neg9(R);
    }
  }
}

} /* namespace idl */