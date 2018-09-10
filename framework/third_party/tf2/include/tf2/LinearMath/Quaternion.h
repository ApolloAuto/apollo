/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef TF2_QUATERNION_H_
#define TF2_QUATERNION_H_


#include "Vector3.h"
#include "QuadWord.h"

namespace tf2
{

/**@brief The Quaternion implements quaternion to perform linear algebra rotations in combination with Matrix3x3, Vector3 and Transform. */
class Quaternion : public QuadWord {
public:
  /**@brief No initialization constructor */
	Quaternion() {}

	//		template <typename tf2Scalar>
	//		explicit Quaternion(const tf2Scalar *v) : Tuple4<tf2Scalar>(v) {}
  /**@brief Constructor from scalars */
	Quaternion(const tf2Scalar& x, const tf2Scalar& y, const tf2Scalar& z, const tf2Scalar& w) 
		: QuadWord(x, y, z, w) 
	{}
  /**@brief Axis angle Constructor
   * @param axis The axis which the rotation is around
   * @param angle The magnitude of the rotation around the angle (Radians) */
	Quaternion(const Vector3& axis, const tf2Scalar& angle) 
	{ 
		setRotation(axis, angle); 
	}
  /**@brief Constructor from Euler angles
   * @param yaw Angle around Y unless TF2_EULER_DEFAULT_ZYX defined then Z
   * @param pitch Angle around X unless TF2_EULER_DEFAULT_ZYX defined then Y
   * @param roll Angle around Z unless TF2_EULER_DEFAULT_ZYX defined then X */
  Quaternion(const tf2Scalar& yaw, const tf2Scalar& pitch, const tf2Scalar& roll) __attribute__((deprecated))
	{ 
#ifndef TF2_EULER_DEFAULT_ZYX
		setEuler(yaw, pitch, roll); 
#else
		setRPY(roll, pitch, yaw);
#endif 
	}
  /**@brief Set the rotation using axis angle notation 
   * @param axis The axis around which to rotate
   * @param angle The magnitude of the rotation in Radians */
	void setRotation(const Vector3& axis, const tf2Scalar& angle)
	{
		tf2Scalar d = axis.length();
		tf2Assert(d != tf2Scalar(0.0));
		tf2Scalar s = tf2Sin(angle * tf2Scalar(0.5)) / d;
		setValue(axis.x() * s, axis.y() * s, axis.z() * s, 
			tf2Cos(angle * tf2Scalar(0.5)));
	}
  /**@brief Set the quaternion using Euler angles
   * @param yaw Angle around Y
   * @param pitch Angle around X
   * @param roll Angle around Z */
	void setEuler(const tf2Scalar& yaw, const tf2Scalar& pitch, const tf2Scalar& roll)
	{
		tf2Scalar halfYaw = tf2Scalar(yaw) * tf2Scalar(0.5);  
		tf2Scalar halfPitch = tf2Scalar(pitch) * tf2Scalar(0.5);  
		tf2Scalar halfRoll = tf2Scalar(roll) * tf2Scalar(0.5);  
		tf2Scalar cosYaw = tf2Cos(halfYaw);
		tf2Scalar sinYaw = tf2Sin(halfYaw);
		tf2Scalar cosPitch = tf2Cos(halfPitch);
		tf2Scalar sinPitch = tf2Sin(halfPitch);
		tf2Scalar cosRoll = tf2Cos(halfRoll);
		tf2Scalar sinRoll = tf2Sin(halfRoll);
		setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
			cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
			sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
			cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
	}
  /**@brief Set the quaternion using fixed axis RPY
   * @param roll Angle around X 
   * @param pitch Angle around Y
   * @param yaw Angle around Z*/
  void setRPY(const tf2Scalar& roll, const tf2Scalar& pitch, const tf2Scalar& yaw)
	{
		tf2Scalar halfYaw = tf2Scalar(yaw) * tf2Scalar(0.5);  
		tf2Scalar halfPitch = tf2Scalar(pitch) * tf2Scalar(0.5);  
		tf2Scalar halfRoll = tf2Scalar(roll) * tf2Scalar(0.5);  
		tf2Scalar cosYaw = tf2Cos(halfYaw);
		tf2Scalar sinYaw = tf2Sin(halfYaw);
		tf2Scalar cosPitch = tf2Cos(halfPitch);
		tf2Scalar sinPitch = tf2Sin(halfPitch);
		tf2Scalar cosRoll = tf2Cos(halfRoll);
		tf2Scalar sinRoll = tf2Sin(halfRoll);
		setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw, //x
                         cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw, //y
                         cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw, //z
                         cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw); //formerly yzx
	}
  /**@brief Set the quaternion using euler angles 
   * @param yaw Angle around Z
   * @param pitch Angle around Y
   * @param roll Angle around X */
  void setEulerZYX(const tf2Scalar& yaw, const tf2Scalar& pitch, const tf2Scalar& roll) __attribute__((deprecated))
	{
          setRPY(roll, pitch, yaw);
	}
  /**@brief Add two quaternions
   * @param q The quaternion to add to this one */
	TF2SIMD_FORCE_INLINE	Quaternion& operator+=(const Quaternion& q)
	{
		m_floats[0] += q.x(); m_floats[1] += q.y(); m_floats[2] += q.z(); m_floats[3] += q.m_floats[3];
		return *this;
	}

  /**@brief Sutf2ract out a quaternion
   * @param q The quaternion to sutf2ract from this one */
	Quaternion& operator-=(const Quaternion& q) 
	{
		m_floats[0] -= q.x(); m_floats[1] -= q.y(); m_floats[2] -= q.z(); m_floats[3] -= q.m_floats[3];
		return *this;
	}

  /**@brief Scale this quaternion
   * @param s The scalar to scale by */
	Quaternion& operator*=(const tf2Scalar& s)
	{
		m_floats[0] *= s; m_floats[1] *= s; m_floats[2] *= s; m_floats[3] *= s;
		return *this;
	}

  /**@brief Multiply this quaternion by q on the right
   * @param q The other quaternion 
   * Equivilant to this = this * q */
	Quaternion& operator*=(const Quaternion& q)
	{
		setValue(m_floats[3] * q.x() + m_floats[0] * q.m_floats[3] + m_floats[1] * q.z() - m_floats[2] * q.y(),
			m_floats[3] * q.y() + m_floats[1] * q.m_floats[3] + m_floats[2] * q.x() - m_floats[0] * q.z(),
			m_floats[3] * q.z() + m_floats[2] * q.m_floats[3] + m_floats[0] * q.y() - m_floats[1] * q.x(),
			m_floats[3] * q.m_floats[3] - m_floats[0] * q.x() - m_floats[1] * q.y() - m_floats[2] * q.z());
		return *this;
	}
  /**@brief Return the dot product between this quaternion and another
   * @param q The other quaternion */
	tf2Scalar dot(const Quaternion& q) const
	{
		return m_floats[0] * q.x() + m_floats[1] * q.y() + m_floats[2] * q.z() + m_floats[3] * q.m_floats[3];
	}

  /**@brief Return the length squared of the quaternion */
	tf2Scalar length2() const
	{
		return dot(*this);
	}

  /**@brief Return the length of the quaternion */
	tf2Scalar length() const
	{
		return tf2Sqrt(length2());
	}

  /**@brief Normalize the quaternion 
   * Such that x^2 + y^2 + z^2 +w^2 = 1 */
	Quaternion& normalize() 
	{
		return *this /= length();
	}

  /**@brief Return a scaled version of this quaternion
   * @param s The scale factor */
	TF2SIMD_FORCE_INLINE Quaternion
	operator*(const tf2Scalar& s) const
	{
		return Quaternion(x() * s, y() * s, z() * s, m_floats[3] * s);
	}


  /**@brief Return an inversely scaled versionof this quaternion
   * @param s The inverse scale factor */
	Quaternion operator/(const tf2Scalar& s) const
	{
		tf2Assert(s != tf2Scalar(0.0));
		return *this * (tf2Scalar(1.0) / s);
	}

  /**@brief Inversely scale this quaternion
   * @param s The scale factor */
	Quaternion& operator/=(const tf2Scalar& s) 
	{
		tf2Assert(s != tf2Scalar(0.0));
		return *this *= tf2Scalar(1.0) / s;
	}

  /**@brief Return a normalized version of this quaternion */
	Quaternion normalized() const 
	{
		return *this / length();
	} 
  /**@brief Return the ***half*** angle between this quaternion and the other 
   * @param q The other quaternion */
	tf2Scalar angle(const Quaternion& q) const 
	{
		tf2Scalar s = tf2Sqrt(length2() * q.length2());
		tf2Assert(s != tf2Scalar(0.0));
		return tf2Acos(dot(q) / s);
	}
	/**@brief Return the angle between this quaternion and the other along the shortest path
	* @param q The other quaternion */
	tf2Scalar angleShortestPath(const Quaternion& q) const 
	{
		tf2Scalar s = tf2Sqrt(length2() * q.length2());
		tf2Assert(s != tf2Scalar(0.0));
		if (dot(q) < 0) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
			return tf2Acos(dot(-q) / s) * tf2Scalar(2.0);
		else 
			return tf2Acos(dot(q) / s) * tf2Scalar(2.0);
	}
        /**@brief Return the angle [0, 2Pi] of rotation represented by this quaternion */
	tf2Scalar getAngle() const 
	{
		tf2Scalar s = tf2Scalar(2.) * tf2Acos(m_floats[3]);
		return s;
	}

        /**@brief Return the angle [0, Pi] of rotation represented by this quaternion along the shortest path */
	tf2Scalar getAngleShortestPath() const 
	{
	tf2Scalar s;
		if (m_floats[3] >= 0)
			s = tf2Scalar(2.) * tf2Acos(m_floats[3]);
		else
			s = tf2Scalar(2.) * tf2Acos(-m_floats[3]);

		return s;
	}

	/**@brief Return the axis of the rotation represented by this quaternion */
	Vector3 getAxis() const
	{
		tf2Scalar s_squared = tf2Scalar(1.) - tf2Pow(m_floats[3], tf2Scalar(2.));
		if (s_squared < tf2Scalar(10.) * TF2SIMD_EPSILON) //Check for divide by zero
			return Vector3(1.0, 0.0, 0.0);  // Arbitrary
		tf2Scalar s = tf2Sqrt(s_squared);
		return Vector3(m_floats[0] / s, m_floats[1] / s, m_floats[2] / s);
	}

	/**@brief Return the inverse of this quaternion */
	Quaternion inverse() const
	{
		return Quaternion(-m_floats[0], -m_floats[1], -m_floats[2], m_floats[3]);
	}

  /**@brief Return the sum of this quaternion and the other 
   * @param q2 The other quaternion */
	TF2SIMD_FORCE_INLINE Quaternion
	operator+(const Quaternion& q2) const
	{
		const Quaternion& q1 = *this;
		return Quaternion(q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z(), q1.m_floats[3] + q2.m_floats[3]);
	}

  /**@brief Return the difference between this quaternion and the other 
   * @param q2 The other quaternion */
	TF2SIMD_FORCE_INLINE Quaternion
	operator-(const Quaternion& q2) const
	{
		const Quaternion& q1 = *this;
		return Quaternion(q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z(), q1.m_floats[3] - q2.m_floats[3]);
	}

  /**@brief Return the negative of this quaternion 
   * This simply negates each element */
	TF2SIMD_FORCE_INLINE Quaternion operator-() const
	{
		const Quaternion& q2 = *this;
		return Quaternion( - q2.x(), - q2.y(),  - q2.z(),  - q2.m_floats[3]);
	}
  /**@todo document this and it's use */
	TF2SIMD_FORCE_INLINE Quaternion farthest( const Quaternion& qd) const 
	{
		Quaternion diff,sum;
		diff = *this - qd;
		sum = *this + qd;
		if( diff.dot(diff) > sum.dot(sum) )
			return qd;
		return (-qd);
	}

	/**@todo document this and it's use */
	TF2SIMD_FORCE_INLINE Quaternion nearest( const Quaternion& qd) const 
	{
		Quaternion diff,sum;
		diff = *this - qd;
		sum = *this + qd;
		if( diff.dot(diff) < sum.dot(sum) )
			return qd;
		return (-qd);
	}


  /**@brief Return the quaternion which is the result of Spherical Linear Interpolation between this and the other quaternion
   * @param q The other quaternion to interpolate with 
   * @param t The ratio between this and q to interpolate.  If t = 0 the result is this, if t=1 the result is q.
   * Slerp interpolates assuming constant velocity.  */
	Quaternion slerp(const Quaternion& q, const tf2Scalar& t) const
	{
          tf2Scalar theta = angleShortestPath(q) / tf2Scalar(2.0);
		if (theta != tf2Scalar(0.0))
		{
			tf2Scalar d = tf2Scalar(1.0) / tf2Sin(theta);
			tf2Scalar s0 = tf2Sin((tf2Scalar(1.0) - t) * theta);
			tf2Scalar s1 = tf2Sin(t * theta);   
                        if (dot(q) < 0) // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
                          return Quaternion((m_floats[0] * s0 + -q.x() * s1) * d,
                                              (m_floats[1] * s0 + -q.y() * s1) * d,
                                              (m_floats[2] * s0 + -q.z() * s1) * d,
                                              (m_floats[3] * s0 + -q.m_floats[3] * s1) * d);
                        else
                          return Quaternion((m_floats[0] * s0 + q.x() * s1) * d,
                                              (m_floats[1] * s0 + q.y() * s1) * d,
                                              (m_floats[2] * s0 + q.z() * s1) * d,
                                              (m_floats[3] * s0 + q.m_floats[3] * s1) * d);
                        
		}
		else
		{
			return *this;
		}
	}

	static const Quaternion&	getIdentity()
	{
		static const Quaternion identityQuat(tf2Scalar(0.),tf2Scalar(0.),tf2Scalar(0.),tf2Scalar(1.));
		return identityQuat;
	}

	TF2SIMD_FORCE_INLINE const tf2Scalar& getW() const { return m_floats[3]; }

	
};


/**@brief Return the negative of a quaternion */
TF2SIMD_FORCE_INLINE Quaternion
operator-(const Quaternion& q)
{
	return Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
}



/**@brief Return the product of two quaternions */
TF2SIMD_FORCE_INLINE Quaternion
operator*(const Quaternion& q1, const Quaternion& q2) {
	return Quaternion(q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
		q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z(),
		q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x(),
		q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z()); 
}

TF2SIMD_FORCE_INLINE Quaternion
operator*(const Quaternion& q, const Vector3& w)
{
	return Quaternion( q.w() * w.x() + q.y() * w.z() - q.z() * w.y(),
		q.w() * w.y() + q.z() * w.x() - q.x() * w.z(),
		q.w() * w.z() + q.x() * w.y() - q.y() * w.x(),
		-q.x() * w.x() - q.y() * w.y() - q.z() * w.z()); 
}

TF2SIMD_FORCE_INLINE Quaternion
operator*(const Vector3& w, const Quaternion& q)
{
	return Quaternion( w.x() * q.w() + w.y() * q.z() - w.z() * q.y(),
		w.y() * q.w() + w.z() * q.x() - w.x() * q.z(),
		w.z() * q.w() + w.x() * q.y() - w.y() * q.x(),
		-w.x() * q.x() - w.y() * q.y() - w.z() * q.z()); 
}

/**@brief Calculate the dot product between two quaternions */
TF2SIMD_FORCE_INLINE tf2Scalar 
dot(const Quaternion& q1, const Quaternion& q2) 
{ 
	return q1.dot(q2); 
}


/**@brief Return the length of a quaternion */
TF2SIMD_FORCE_INLINE tf2Scalar
length(const Quaternion& q) 
{ 
	return q.length(); 
}

/**@brief Return the ***half*** angle between two quaternions*/
TF2SIMD_FORCE_INLINE tf2Scalar
angle(const Quaternion& q1, const Quaternion& q2) 
{ 
	return q1.angle(q2); 
}

/**@brief Return the shortest angle between two quaternions*/
TF2SIMD_FORCE_INLINE tf2Scalar
angleShortestPath(const Quaternion& q1, const Quaternion& q2) 
{ 
	return q1.angleShortestPath(q2); 
}

/**@brief Return the inverse of a quaternion*/
TF2SIMD_FORCE_INLINE Quaternion
inverse(const Quaternion& q) 
{
	return q.inverse();
}

/**@brief Return the result of spherical linear interpolation betwen two quaternions 
 * @param q1 The first quaternion
 * @param q2 The second quaternion 
 * @param t The ration between q1 and q2.  t = 0 return q1, t=1 returns q2 
 * Slerp assumes constant velocity between positions. */
TF2SIMD_FORCE_INLINE Quaternion
slerp(const Quaternion& q1, const Quaternion& q2, const tf2Scalar& t) 
{
	return q1.slerp(q2, t);
}

TF2SIMD_FORCE_INLINE Vector3 
quatRotate(const Quaternion& rotation, const Vector3& v) 
{
	Quaternion q = rotation * v;
	q *= rotation.inverse();
	return Vector3(q.getX(),q.getY(),q.getZ());
}

TF2SIMD_FORCE_INLINE Quaternion 
shortestArcQuat(const Vector3& v0, const Vector3& v1) // Game Programming Gems 2.10. make sure v0,v1 are normalized
{
	Vector3 c = v0.cross(v1);
	tf2Scalar  d = v0.dot(v1);

	if (d < -1.0 + TF2SIMD_EPSILON)
	{
		Vector3 n,unused;
		tf2PlaneSpace1(v0,n,unused);
		return Quaternion(n.x(),n.y(),n.z(),0.0f); // just pick any vector that is orthogonal to v0
	}

	tf2Scalar  s = tf2Sqrt((1.0f + d) * 2.0f);
	tf2Scalar rs = 1.0f / s;

	return Quaternion(c.getX()*rs,c.getY()*rs,c.getZ()*rs,s * 0.5f);
}

TF2SIMD_FORCE_INLINE Quaternion 
shortestArcQuatNormalize2(Vector3& v0,Vector3& v1)
{
	v0.normalize();
	v1.normalize();
	return shortestArcQuat(v0,v1);
}

}
#endif




