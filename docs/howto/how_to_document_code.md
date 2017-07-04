# How to document source code for doxygen

Developers who are not familiar with doxygen can get more information from its official website [http://www.stack.nl/~dimitri/doxygen/](http://www.stack.nl/~dimitri/doxygen/).

We use [time.h](https://github.com/ApolloAuto/apollo/blob/master/modules/common/time/time.h) as an example to explain how to document.
## file
```
/**
 * @file
 *
 * @brief This library provides the utilities to deal with timestamps.
 * currently our assumption is that every timestamp will be of a
 * precision at 1 us.
 */
```

## namespace
```
/**
 * @namespace apollo::common::time
 * @brief apollo::common::time
 */
namespace apollo {
namespace common {
namespace time {
```

## class
```
/**
 * @class Clock
 * @brief a singleton clock that can be used to get the current current
 * timestamp. The source can be either system clock or a mock clock.
 * Mock clock is for testing purpose mainly. The mock clock related
 * methods are not thread-safe.
 */
class Clock {
 public:
 ...
```

## function
```
/**
  * @brief Set the behavior of the \class Clock.
  * @param is_system_clock if provided with value TRUE, further call
  * to Now() will return timestamp based on the system clock. If
  * provided with FALSE, it will use the mock clock instead.
  */
 static void UseSystemClock(bool is_system_clock) {
   Clock::instance()->is_system_clock_ = is_system_clock;
 }
```

## public / protected class member variables
```
/// Stores the currently set timestamp, which serves mock clock
/// queries.
Timestamp mock_now_;
```
