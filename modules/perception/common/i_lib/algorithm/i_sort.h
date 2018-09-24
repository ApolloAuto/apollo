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
#ifndef I_LIB_ALGORITHM_I_SORT_H
#define I_LIB_ALGORITHM_I_SORT_H

#include <assert.h>
#include "../core/i_basic.h"

namespace idl {
/*Strictly less*/
template <typename T>
inline bool i_less(const T &a, const T &b) {
  return a < b;
}

/*Strictly larger*/
template <typename T>
inline bool i_larger(const T &a, const T &b) {
  return a > b;
}

/*Median of three values*/
template <typename T>
inline T i_median3(const T &a, const T &b, const T &c) {
  if (a <= b) {
    if (c > a) {
      if (c <= b) {
        return (c);
      }
      return (b);
    }
    return (a);
  }
  if (c > b) {
    if (c <= a) {
      return (c);
    }
    return (a);
  }
  return (b);
}

/*Insertion sort*/
template <typename T, bool compare(const T &, const T &)>
inline void i_insertion_sort(T *a, int n) {
  int i, j;
  T tmp;
  for (i = 1; i < n; i++) {
    i_move(a[i], tmp);
    for (j = i; j > 0; j--) {
      if (compare(a[j - 1], tmp)) {
        break;
      }
      i_move(a[j - 1], a[j]);
    }
    i_move(tmp, a[j]);
  }
}

/*Move the indexed elements to the left hand side of array "a" in place. Array
  "a" has size "n*element_size".
  Array "indices" has valid size "nr_indexed_elements", its length should be >=
  "nr_indexed_elements".
  Array a and indices will be destroyed after calling this routine*/
template <typename T>
inline void i_indexed_shuffle(T *a, int *indices, int n, int element_size,
                              int nr_indexed_elements,
                              bool is_sort_indices = true) {
  /*pre-sort indices array into ascending order*/
  if (is_sort_indices) {
    i_insertion_sort<int, i_less>(indices, nr_indexed_elements);
  }
  /*move indexed elements to the left hand side of a*/
  int i, j, m = i_min(n, nr_indexed_elements);
  for (i = 0; i < m; ++i) {
    j = indices[i];
    if (j != i) {
      i_swap(a + i * element_size, a + j * element_size,
             element_size); /*swap indexed elements to the left*/
    }
  }
}

/*Move the indexed elements to the left hand side of array "a" and "b" in place.
Array "a" has size "n*element_size_a".
Array "b" has size "n*element_size_b".
Array "indices" has valid size "nr_indexed_elements", its length should be >=
"nr_indexed_elements".
Array a, b, and indices will be destroyed after calling this routine*/
template <typename T>
inline void i_indexed_shuffle(T *a, T *b, int *indices, int n,
                              int element_size_a, int element_size_b,
                              int nr_indexed_elements,
                              bool is_sort_indices = true) {
  /*pre-sort indices array into ascending order*/
  if (n <= 1) {
    return;
  }
  if (is_sort_indices) {
    i_insertion_sort<int, i_less>(indices, nr_indexed_elements);
  }
  /*move indexed elements to the left hand side of a*/
  int i, j, m = i_min(n, nr_indexed_elements);
  for (i = 0; i < m; ++i) {
    j = indices[i];
    if (j != i) {
      i_swap(a + i * element_size_a, a + j * element_size_a,
             element_size_a); /*swap indexed elements to the left*/
      i_swap(b + i * element_size_b, b + j * element_size_b,
             element_size_b); /*swap indexed elements to the left*/
    }
  }
}

/*Move the indexed elements to the left hand side of array "a" and "b" in place.
Array "a" has size "n*2".
Array "b" has size "n*2".
Array "indices" has valid size "nr_indexed_elements", its length should be >=
"nr_indexed_elements".
Array a, b, and indices will be destroyed after calling this routine*/
template <typename T>
inline void i_indexed_shuffle2(T *a, T *b, int *indices, int n,
                               int nr_indexed_elements,
                               bool is_sort_indices = true) {
  /*pre-sort indices array into ascending order*/
  if (n <= 1) {
    return;
  }
  if (is_sort_indices) {
    i_insertion_sort<int, i_less>(indices, nr_indexed_elements);
  }
  /*move indexed elements to the left hand side of a*/
  int i, j, i2, j2, m = i_min(n, nr_indexed_elements);
  for (i = 0; i < m; ++i) {
    j = indices[i];
    if (j != i) {
      i2 = (i << 1);
      j2 = (j << 1);
      i_swap2(a + i2, a + j2); /*swap indexed elements to the left*/
      i_swap2(b + i2, b + j2); /*swap indexed elements to the left*/
    }
  }
}

/*Move the indexed elements to the left hand side of array "a" and "b" in place.
Array "a" has size "n*3".
Array "b" has size "n*3".
Array "indices" has valid size "nr_indexed_elements", its length should be >=
"nr_indexed_elements".
Array a, b, and indices will be destroyed after calling this routine*/
template <typename T>
inline void i_indexed_shuffle3(T *a, T *b, int *indices, int n,
                               int nr_indexed_elements,
                               bool is_sort_indices = true) {
  /*pre-sort indices array into ascending order*/
  if (n <= 1) {
    return;
  }
  if (is_sort_indices) {
    i_insertion_sort<int, i_less>(indices, nr_indexed_elements);
  }
  /*move indexed elements to the left hand side of a*/
  int i, j, i3, j3, m = i_min(n, nr_indexed_elements);
  for (i = 0; i < m; ++i) {
    j = indices[i];
    if (j != i) {
      i3 = i * 3;
      j3 = j * 3;
      i_swap3(a + i3, a + j3); /*swap indexed elements to the left*/
      i_swap3(b + i3, b + j3); /*swap indexed elements to the left*/
    }
  }
}

/*Move segment [first,last] from src to dst, which are both n element arrays,
and split based on pivot element pivot
return position of first and last element equal to pivot
For efficiency, do NOT move the elements equal to pivot*/
template <typename T>
inline void i_quick_split(const T *src, T *dst, int n, T pivot, int first,
                          int last, long &first_equal, long &last_equal) {
  assert(first >= 0 && last < n && first <= last);
  T v, *p1, *p2;
  const T *p3, *p4;
  /*start with pointers at the first and last of d and first of s*/
  p1 = dst + first;
  p2 = dst + last;
  p3 = src + first;
  p4 = src + last;
  /*loop until the src is traversed*/
  for (; p3 <= p4;) {
    /*get from src*/
    v = *p3++;
    if (v < pivot) {
      *p1++ = v; /*store low to dst*/
    } else if (v > pivot) {
      *p2-- = v; /*store high to dst*/
    }
  }
  first_equal = (p1 - dst);
  last_equal = (p2 - dst);
}

/*Return the value of element of order k in array a whose length is n,
working array "wa" must be pre-allocated with at least 2n positions*/
template <typename T>
T i_quick_select(T *a, int n, int k, T *wa) {
  int first, last;
  long first_equal, last_equal;
  T pivot = (T)0, *t1, *t2, *s, *d;

  /*Make pointers to two buffers*/
  t1 = wa;
  t2 = wa + n;
  /*Take from a and store to t1*/
  s = a;
  d = t1;

  first = 0;
  last = n - 1;
  while (first <= last) {
    pivot = i_median3(s[first], s[(first + last) >> 1], s[last]);
    if (last <= first) {
      return (pivot);
    }

    /*Split on pivot*/
    i_quick_split(s, d, n, pivot, first, last, first_equal, last_equal);

    if (k < first_equal) {
      last = first_equal - 1;
    } else {
      if (k > last_equal) {
        first = last_equal + 1;
      } else
        return (pivot);
    }

    /*Take from t1 and store to t2*/
    s = t1;
    d = t2;
    /*Swap for the next iteration*/
    i_swap<T *>(t1, t2);
  }
  return (pivot); /*unreachable! put here to avoid warning*/
}

} /* namespace idl */

#endif