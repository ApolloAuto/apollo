
#pragma once

#include <iostream>
#include <type_traits>

#define DEFINE_MEMBER_CHECKER(member)                                                                         \
  template <typename T, typename V = bool>                                                                    \
  struct has_##member : std::false_type                                                                       \
  {                                                                                                           \
  };                                                                                                          \
  template <typename T>                                                                                       \
  struct has_##member<                                                                                        \
      T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().member), void>::value, bool>::type> \
      : std::true_type                                                                                        \
  {                                                                                                           \
  };

DEFINE_MEMBER_CHECKER(x)
DEFINE_MEMBER_CHECKER(y)
DEFINE_MEMBER_CHECKER(z)
DEFINE_MEMBER_CHECKER(intensity)
DEFINE_MEMBER_CHECKER(ring)
DEFINE_MEMBER_CHECKER(timestamp)

#define VANJEE_HAS_MEMBER(C, member) has_##member<C>::value
template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, x)>::type setX(T_Point &point, const float &value)
{
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, x)>::type setX(T_Point &point, const float &value)
{
  point.x = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, y)>::type setY(T_Point &point, const float &value)
{
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, y)>::type setY(T_Point &point, const float &value)
{
  point.y = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, z)>::type setZ(T_Point &point, const float &value)
{
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, z)>::type setZ(T_Point &point, const float &value)
{
  point.z = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point &point,
                                                                                          const float &value)
{
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point &point,
                                                                                         const float &value)
{
  point.intensity = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point &point, const uint16_t &value)
{
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point &point, const uint16_t &value)
{
  point.ring = value;
}

template <typename T_Point>
inline typename std::enable_if<!VANJEE_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point &point,
                                                                                          const double &value)
{
}

template <typename T_Point>
inline typename std::enable_if<VANJEE_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point &point,
                                                                                         const double &value)
{
  point.timestamp = value;
}
