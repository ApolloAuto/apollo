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
#ifndef I_LIB_GEOMETRY_I_PLANE_H
#define I_LIB_GEOMETRY_I_PLANE_H

#include "../algorithm/i_sort.h"
#include "../core/i_alloc.h"
#include "../core/i_struct.h"
#include "../numeric/i_eig.h"
#include "../numeric/i_lu.h"

namespace idl {
/*General plane equation pi: ax+by+cz+d = 0*/

/*Another 3D Plane data structure:*/
template <typename T>
struct PlanePara {
  PlanePara(const PlanePara<T>& pi) {
    i_copy4(pi.p, p);
    i_copy3(pi.t, t);
    i_copy2(pi.data_stat, data_stat);
  }
  PlanePara& operator=(const PlanePara<T>& pi) {
    i_copy4(pi.p, this->p);
    i_copy3(pi.t, this->t);
    i_copy2(pi.data_stat, this->data_stat);
    return (*this);
  }
  /*first 3 elements: normal with unit norm, last elemet: d*/
  T p[4];
  T t[3]; /*centroid*/
  /*mean distance and standard deviation of
  the point to plane distance distribution*/
  T data_stat[2];
  void clear() {
    i_zero4(p);
    i_zero3(t);
    i_zero2(data_stat);
  }
  void assign(const T pi[9]) {
    i_copy4(pi, p);
    i_copy3(pi + 4, t);
    i_copy2(pi + 7, data_stat);
  }
  bool isvalid() { return (i_sqrt(i_squaresum3(p)) > (T)0); }
  PlanePara() { clear(); }
};

/*Compute the projection (q) of a 3D point (p) on a plane (pi) in 3D space*/
template <typename T>
inline void i_point_on_plane_projection(const T pi[4], const T p[3], T q[3]) {
  T npi[4];
  i_copy4(pi, npi);
  T sf = i_rec(i_l2_norm3(npi));
  i_scale4(npi, sf); /*plane with unit norm*/
  sf = -(i_dot3(npi, p) + npi[3]);
  q[0] = p[0] + sf * npi[0];
  q[1] = p[1] + sf * npi[1];
  q[2] = p[2] + sf * npi[2];
}

/*Measure point to plane distance in 3D space, point p is in inhomogeneous
 * coordinates*/
template <typename T>
inline T i_plane_to_point_distance(const T pi[4], const T p[3]) {
  return i_div(i_abs(i_dot3(pi, p) + pi[3]), i_l2_norm3(pi));
}

/*Measure point to plane distance in 3D space, point p is in inhomogeneous
 * coordinates*/
template <typename T>
inline T i_plane_to_point_distance(const T pi[4], const T p[3], T l2_norm3_pi) {
  return i_div(i_abs(i_dot3(pi, p) + pi[3]), l2_norm3_pi);
}

/*Measure point to plane distance in 3D space, point p is in inhomogeneous
 * coordinates*/
template <typename T>
inline T i_plane_to_point_distance_w_unit_norm(const T pi[4], const T p[3]) {
  return i_abs(i_dot3(pi, p) + pi[3]);
}

/*Measure point to plane distance (signed) in 3D space, point p is in
inhomogeneous coordinates
distance is positive if p is on the same side of the plane pi as the normal
vector and negative if it is on the opposite side*/
template <typename T>
inline T i_plane_to_point_signed_distance_w_unit_norm(const T pi[4],
                                                      const T p[3]) {
  return i_dot3(pi, p) + pi[3];
}

/*Measure point to plane distance in 3D space, point p is in inhomogeneous
 * coordinates*/
template <typename T>
inline T i_plane_to_point_distance_w_normalized_plane_norm(const T pi[4],
                                                           const T p[3]) {
  return i_abs(i_dot3(pi, p) + pi[3]);
}

/*Measure the normal angle in degree between two planes*/
template <typename T>
inline T i_plane_to_plane_normal_delta_degree_z_up(const T pi_p[4],
                                                   const T pi_q[4]) {
  T normal_p[3], normal_q[3];
  i_copy3(pi_p, normal_p);
  i_copy3(pi_q, normal_q);

  /*Z is up*/
  if (normal_p[2] < (T)0.0) {
    i_neg3(normal_p);
  }
  if (normal_q[2] < (T)0.0) {
    i_neg3(normal_q);
  }

  i_scale3(normal_p, i_rec(i_sqrt(i_squaresum3(normal_p)))); /*normalize*/
  i_scale3(normal_q, i_rec(i_sqrt(i_squaresum3(normal_q)))); /*normalize*/
  return i_radians_to_degree(i_acos(i_dot3(normal_p, normal_q)));
}

/*Fit a plane pi (ax+by+cz+d = 0): in 3D space using
total least square method (Orthogonal Distance Regression). The input n 3D
points are in
inhomogeneous coordinates. Array X has size n * 3 and points are stored as[x0,
y0, z0, x1, y1, z1, ...].
pi is stored as 4 - vector[a, b, c, d]. x will be destroyed after calling this
routine.*/
template <typename T>
inline void i_plane_fit_total_least_square(T* X, T pi[4], int n) {
  i_zero4(pi);
  if (n < 3) {
    return;
  }
  int i, j;
  T mat_a[9], eigv[3], mat_q[9];
  /*compute the centroid of input data points*/
  T xm = (T)0.0;
  T ym = (T)0.0;
  T zm = (T)0.0;
  for (i = 0, j = 0; i < n; i++) {
    xm += X[j++];
    ym += X[j++];
    zm += X[j++];
  }
  xm /= n;
  ym /= n;
  zm /= n;
  for (i = 0, j = 0; i < n; i++) {
    X[j++] -= xm;
    X[j++] -= ym;
    X[j++] -= zm;
  }
  i_mult_AtA_nx3(X, mat_a, n);
  i_eig_symmetric_3x3_closed(mat_a, eigv, mat_q);
  /*pi's normal vector is the eigen vector of
  mat_a corresponding to its smallest eigen value*/
  pi[0] = mat_q[2];
  pi[1] = mat_q[5];
  pi[2] = mat_q[8];
  /*the optimal plane should pass [xm, ym, zm]:*/
  pi[3] = -(pi[0] * xm + pi[1] * ym + pi[2] * zm);
}

/*Fit a plane pi (ax+by+cz+d = 0): in 3D space using
total least square method (Orthogonal Distance Regression). The input n 3D
points are in
inhomogeneous coordinates. Array X has size 3 * 3 and points are stored as[x0,
y0, z0, x1, y1, z1, ...].
pi is stored as 4 - vector[a, b, c, d]. x will be destroyed after calling this
routine.*/
template <typename T>
inline void i_plane_fit_total_least_square3(T X[9], T pi[4]) {
  T mat_a[9], eigv[3], mat_q[9];
  /*compute the centroid of input data points*/
  i_zero4(pi);
  T xm = (X[0] + X[3] + X[6]) / 3;
  T ym = (X[1] + X[4] + X[7]) / 3;
  T zm = (X[2] + X[5] + X[8]) / 3;
  X[0] -= xm;
  X[1] -= ym;
  X[2] -= zm;
  X[3] -= xm;
  X[4] -= ym;
  X[5] -= zm;
  X[6] -= xm;
  X[7] -= ym;
  X[8] -= zm;
  i_mult_AtA_3x3(X, mat_a);
  i_eig_symmetric_3x3_closed(mat_a, eigv, mat_q);
  /*pi's normal vector is the eigen vector of
  mat_a corresponding to its smallest eigen value*/
  pi[0] = mat_q[2];
  pi[1] = mat_q[5];
  pi[2] = mat_q[8];
  /*the optimal plane should pass [xm, ym, zm]:*/
  pi[3] = -(pi[0] * xm + pi[1] * ym + pi[2] * zm);
}

/*Fit a plane pi: {[a,b,c,d], [x,y,z]} in 3D space using
total least square method (Orthogonal Distance Regression). The input n 3D
points are in
inhomogeneous coordinates. Array X has size n * 3 and points are stored as[x0,
y0, z0, x1, y1, z1, ...].
The routine needs n * 3 positions of scratch space in Xp.*/
template <typename T>
inline void i_plane_fit_total_least_square(
    const T* X, T pi[4], int n, T* Xp, T centroid[3],
    T err_stat[2] /*mean err and err std*/) {
  i_zero4(pi);
  if (centroid != NULL) {
    i_zero3(centroid);
  }
  if (err_stat != NULL) {
    i_zero2(err_stat);
  }
  if (n < 3) {
    return;
  }

  int i, j, n3 = n * 3;
  T mat_a[9], eigv[3], mat_q[9], t[3];
  /*compute the centroid of input data points*/
  T xm = (T)0.0;
  T ym = (T)0.0;
  T zm = (T)0.0;
  for (i = 0, j = 0; i < n; i++) {
    xm += X[j++];
    ym += X[j++];
    zm += X[j++];
  }
  t[0] = xm / n;
  t[1] = ym / n;
  t[2] = zm / n;
  for (i = 0; i < n3; i += 3) {
    i_sub3(X + i, t, Xp + i);
  }
  i_mult_AtA_nx3(Xp, mat_a, n);
  i_eig_symmetric_3x3_closed(mat_a, eigv, mat_q);
  /*pi's normal vector is the eigen vector of
  mat_a corresponding to its smallest eigen value*/
  pi[0] = mat_q[2];
  pi[1] = mat_q[5];
  pi[2] = mat_q[8];
  i_unitize3(pi);
  /*the optimal plane should pass [xm, ym, zm]:*/
  pi[3] = -i_dot3(pi, t);

  if (centroid != NULL) {
    i_copy3(t, centroid);
  }

  /*data stat:*/
  if (err_stat != NULL) {
    const T* cptr_data = X;
    for (i = 0; i < n; ++i) {
      Xp[i] = i_plane_to_point_distance(pi, cptr_data);
      cptr_data += 3;
    }
    err_stat[0] = i_mean(Xp, n);             /*mean*/
    err_stat[1] = i_sdv(Xp, err_stat[0], n); /*sdv*/
  }
}

/*Fit a plane pi: {[a,b,c,d], [x,y,z]} in 3D space using
total least square method (Orthogonal Distance Regression). The input n 3D
points are in
inhomogeneous coordinates. Array X has size n * 3 and points are stored as[x0,
y0, z0, x1, y1, z1, ...].
The routine needs n * 3 positions of scratch space in Xp.*/
template <typename T>
inline void i_plane_fit_total_least_square(const T* X, PlanePara<T>& pi, T* Xp,
                                           int n) {
  pi.clear();
  if (n < 3) {
    return;
  }
  int i, j;
  T mat_a[9], eigv[3], mat_q[9];
  /*compute the centroid of input data points*/
  T xm = (T)0.0;
  T ym = (T)0.0;
  T zm = (T)0.0;
  for (i = 0, j = 0; i < n; i++) {
    xm += X[j++];
    ym += X[j++];
    zm += X[j++];
  }
  xm /= n;
  ym /= n;
  zm /= n;
  for (i = 0, j = 0; i < n; i++, j += 3) {
    Xp[j] = X[j] - xm;
    Xp[j + 1] = X[j + 1] - ym;
    Xp[j + 2] = X[j + 2] - zm;
  }
  i_mult_AtA_nx3(Xp, mat_a, n);
  i_eig_symmetric_3x3_closed(mat_a, eigv, mat_q);
  /*pi's normal vector is the eigen vector of
  mat_a corresponding to its smallest eigen value*/
  pi.p[0] = mat_q[2];
  pi.p[1] = mat_q[5];
  pi.p[2] = mat_q[8];
  i_unitize3(pi.p);
  /*the optimal plane should pass [xm, ym, zm]:*/
  pi.t[0] = xm;
  pi.t[1] = ym;
  pi.t[2] = zm;
  pi.p[3] = -i_dot3(pi.p, pi.t);
  /*data stat:*/
  i_zero2(pi.data_stat);
  const T* cptr_data = X;
  for (i = 0; i < n; ++i) {
    Xp[i] = i_plane_to_point_distance(pi.p, cptr_data);
    cptr_data += 3;
  }
  pi.data_stat[0] = i_mean(Xp, n);
  pi.data_stat[1] = i_sdv(Xp, pi.data_stat[0], n);
}

/*Fit a plane pi (ax+by+cz+d = 0): in 3D space using
total least square method (Orthogonal Distance Regression). The input n 3D
points are in
inhomogeneous coordinates. Array X has size n * 3 and points are stored as[x0,
y0, z0, x1, y1, z1, ...].
pi is stored as 4 - vector[a, b, c, d]. x and indices will be destroyed after
calling this routine.*/
template <typename T>
inline void i_plane_fit_total_least_square(T* X, int* indices, T pi[4],
                                           int m /*total # of samples*/,
                                           int n /*inlier sample_size*/) {
  i_zero4(pi);
  if (n < 3 || n > m) {
    return;
  }
  i_indexed_shuffle(X, indices, m, 3, n);
  int i, j;
  T mat_a[9], eigv[3], mat_q[9];
  /*compute the centroid of input data points*/
  T xm = (T)0.0;
  T ym = (T)0.0;
  T zm = (T)0.0;
  for (i = 0, j = 0; i < n; i++) {
    xm += X[j++];
    ym += X[j++];
    zm += X[j++];
  }
  xm /= n;
  ym /= n;
  zm /= n;
  for (i = 0, j = 0; i < n; i++) {
    X[j++] -= xm;
    X[j++] -= ym;
    X[j++] -= zm;
  }
  i_mult_AtA_nx3(X, mat_a, n);
  i_eig_symmetric_3x3_closed(mat_a, eigv, mat_q);
  /*pi's normal vector is the eigen vector of
  mat_a corresponding to its smallest eigen value*/
  pi[0] = mat_q[2];
  pi[1] = mat_q[5];
  pi[2] = mat_q[8];
  /*the optimal plane should pass [xm, ym, zm]:*/
  pi[3] = -(pi[0] * xm + pi[1] * ym + pi[2] * zm);
}

/*Fit a plane pi (ax+by+cz+d = 0): in 3D space using
total least square method (Orthogonal Distance Regression). The input n 3D
points are in
inhomogeneous coordinates. Array X has size n * 3 and points are stored as[x0,
y0, z0, x1, y1, z1, ...].
pi is stored as 4 - vector[a, b, c, d]. x and indices will be destroyed after
calling this routine.*/
template <typename T>
inline void i_plane_fit_total_least_square(T* X, int* indices, T pi[4],
                                           T centroid[3],
                                           int m /*total # of samples*/,
                                           int n /*inlier sample_size*/) {
  i_zero4(pi);
  if (n < 3 || n > m) {
    return;
  }
  i_indexed_shuffle(X, indices, m, 3, n);
  int i, j;
  T mat_a[9], eigv[3], mat_q[9];
  /*compute the centroid of input data points*/
  T xm = (T)0.0;
  T ym = (T)0.0;
  T zm = (T)0.0;
  for (i = 0, j = 0; i < n; i++) {
    xm += X[j++];
    ym += X[j++];
    zm += X[j++];
  }
  xm /= n;
  ym /= n;
  zm /= n;
  for (i = 0, j = 0; i < n; i++) {
    X[j++] -= xm;
    X[j++] -= ym;
    X[j++] -= zm;
  }
  i_mult_AtA_nx3(X, mat_a, n);
  i_eig_symmetric_3x3_closed(mat_a, eigv, mat_q);
  /*pi's normal vector is the eigen vector of
  mat_a corresponding to its smallest eigen value*/
  pi[0] = mat_q[2];
  pi[1] = mat_q[5];
  pi[2] = mat_q[8];
  /*the optimal plane should pass [xm, ym, zm]:*/
  pi[3] = -(pi[0] * xm + pi[1] * ym + pi[2] * zm);

  if (centroid != NULL) {
    centroid[0] = xm;
    centroid[1] = ym;
    centroid[2] = zm;
  }
}

/*Fit a plane i: {[a,b,c,d], [x,y,z]}  in 3D space using
total least square method (Orthogonal Distance Regression). The input n 3D
points are in
inhomogeneous coordinates. Array X has size n * 3 and points are stored as[x0,
y0, z0, x1, y1, z1, ...].*/
template <typename T>
inline void i_plane_fit_total_least_square_adv(T* X, int* indices, T para[9],
                                               int m /*total # of samples*/,
                                               int n /*inlier sample_size*/) {
  i_zero9(para);
  if (n < 3 || n > m) {
    return;
  }
  int i, j;
  T* xp = i_alloc<T>(n * 3); /*alloc memory in the function!*/
  for (i = 0; i < n; i++) {
    j = indices[i];
    i_copy3(X + j * 3, xp + i * 3);
  }
  T mat_a[9], eigv[3], mat_q[9];
  /*compute the centroid of input data points*/
  T xm = (T)0.0;
  T ym = (T)0.0;
  T zm = (T)0.0;
  for (i = 0, j = 0; i < n; i++) {
    xm += xp[j++];
    ym += xp[j++];
    zm += xp[j++];
  }
  xm /= n;
  ym /= n;
  zm /= n;
  for (i = 0, j = 0; i < n; i++) {
    xp[j++] -= xm;
    xp[j++] -= ym;
    xp[j++] -= zm;
  }
  i_mult_AtA_nx3(xp, mat_a, n);
  i_eig_symmetric_3x3_closed(mat_a, eigv, mat_q);
  /*pi's normal vector is the eigen vector of
  mat_a corresponding to its smallest eigen value*/
  para[0] = mat_q[2];
  para[1] = mat_q[5];
  para[2] = mat_q[8];
  i_unitize3(para);
  /*the optimal plane should pass [xm, ym, zm]:*/
  para[4] = xm;
  para[5] = ym;
  para[6] = zm;
  para[3] = -i_dot3(para, para + 4);
  /*data stat:*/

  for (i = 0; i < n; ++i) {
    j = indices[i];
    xp[i] = i_plane_to_point_distance(para, X + j * 3);
  }
  para[7] = i_mean(xp, n);
  para[8] = i_sdv(xp, para[7], n);
  i_free<T>(xp);
}

/*Fit a plane pi (ax+by+cz+d = 0) with three 3D points in inhomogeneous space -
 * minimal solution*/
template <typename T>
inline void i_plane_fit(const T X1[3], const T X2[3], const T X3[3], T pi[4]) {
  T mat_a[9] = {X1[0], X1[1], X1[2], X2[0], X2[1], X2[2], X3[0], X3[1], X3[2]};
  i_plane_fit_total_least_square(mat_a, pi, 3);
}

/*Fit a plane pi: {[a,b,c,d], [x,y,z]} with three 3D points in inhomogeneous
 * space - minimal solution*/
template <typename T>
inline void i_plane_fit(const T X1[3], const T X2[3], const T X3[3],
                        PlanePara<T>& pi) {
  T mat_a[9] = {X1[0], X1[1], X1[2], X2[0], X2[1], X2[2], X3[0], X3[1], X3[2]};
  T mat_ap[9];
  i_plane_fit_total_least_square(mat_a, pi, mat_ap, 3);
}

/*Fit a plane pi (ax+by+cz+d = 0) with three 3D points in inhomogeneous space -
 * minimal solution*/
template <typename T>
inline void i_plane_fit(const T Xs[9], T pi[4]) {
  T mat_a[9];
  i_copy9(Xs, mat_a);
  i_plane_fit_total_least_square(mat_a, pi, 3);
}

/*Fit a plane pi (ax+by+cz+d = 0) with three 3D points in inhomogeneous space -
minimal solution,
note that the input Xs will be destroyed*/
template <typename T>
inline void i_plane_fit_destroyed(T Xs[9], T pi[4]) {
  i_plane_fit_total_least_square3(Xs, pi);
}

/*Fit a plane pi: {[a,b,c,d], [x,y,z]} with three 3D points in inhomogeneous
 * space - minimal solution*/
template <typename T>
inline void i_plane_fit_adv(const T Xs[9], T para[9]) {
  T mat_a[9], mat_ap[9];
  PlanePara<T> pi;
  i_copy9(Xs, mat_a);
  i_plane_fit_total_least_square(mat_a, pi, mat_ap, 3);
  i_copy4(pi.p, para);
  i_copy3(pi.t, para + 4);
  i_copy2(pi.data_stat, para + 7);
}

/*Compute the cost - sum of point to plane distance (only for inliers) for the
  Ransac routine,
  candidate points are stored in data in inhomogeneous coordinate system*/
template <typename T>
inline void i_plane_fit_ransac_costfunc(const T pi[4], const T* data, int n,
                                        int& nr_liner, int* inliers, T& cost,
                                        T tol) {
  int i, j;
  T plane_to_point_distance = (T)0.0;
  nr_liner = 0;
  cost = (T)0.0;
  for (i = 0, j = 0; i < n; i++, j += 3) {
    plane_to_point_distance = i_plane_to_point_distance(pi, data + j);
    if (plane_to_point_distance < tol) {
      inliers[nr_liner++] = i;
      cost += plane_to_point_distance;
    }
  }
}

} /* namespace idl */

#endif