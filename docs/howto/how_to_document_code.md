# How to Document Source Code in Apollo

Apollo uses [Doxygen](https://www.doxygen.nl/index.html) for source code
documentation. Developers who are not familiar with Doxygen can refer to
official Doxygen Manual(https://www.doxygen.nl/manual/index.html) for an
in-depth knowledge on documenting code with Doxygen. This document serves as a
brief version of
[Doxygen Manual: Documenting the code](https://www.doxygen.nl/manual/docblocks.html))
focusing specificly on C/C++ and Python.

We will take
[modules/common/math/kalman_filter.h](../../modules/common/math/kalman_filter.h)
as an example to show you how to document code the Doxygen way. Note that
Javadoc style is preferred rather than Qt style for comment blocks.

### File

```c++
/**
 * @file
 * @brief Defines the templated KalmanFilter class.
 */
```

### Namespace

```
/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */

namespace apollo {
namespace common {
namespace math {
```

### Class

```
/**
 * @class KalmanFilter
 *
 * @brief Implements a discrete-time Kalman filter.
 *
 * @param XN dimension of state
 * @param ZN dimension of observations
 * @param UN dimension of controls
 */
template <typename T, unsigned int XN, unsigned int ZN, unsigned int UN>
class KalmanFilter {
 public:
 ...
```

### Function

```
  /**
   * @brief Sets the initial state belief distribution.
   *
   * @param x Mean of the state belief distribution
   * @param P Covariance of the state belief distribution
   */
  void SetStateEstimate(const Eigen::Matrix<T, XN, 1> &x,
                        const Eigen::Matrix<T, XN, XN> &P) {
    ...
  }

  /**
   * @brief Get initialization state of the filter
   * @return True if the filter is initialized
   */
  bool IsInitialized() const { return is_initialized_; }

```

### Public / protected class member variables

```
 protected:
  /// Mean of current state belief distribution
  Eigen::Matrix<T, XN, 1> x_;
```

> Note: There is no public/protected member variables for `KalmanFilter`. The
> code above serves for illustration purpose only.
