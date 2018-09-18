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



#ifndef tf2_Transform_H
#define tf2_Transform_H


#include "Matrix3x3.h"


namespace tf2
{

#define TransformData TransformDoubleData


/**@brief The Transform class supports rigid transforms with only translation and rotation and no scaling/shear.
 *It can be used in combination with Vector3, Quaternion and Matrix3x3 linear algebra classes. */
class Transform {
	
  ///Storage for the rotation
	Matrix3x3 m_basis;
  ///Storage for the translation
	Vector3   m_origin;

public:
	
  /**@brief No initialization constructor */
	Transform() {}
  /**@brief Constructor from Quaternion (optional Vector3 )
   * @param q Rotation from quaternion 
   * @param c Translation from Vector (default 0,0,0) */
	explicit TF2SIMD_FORCE_INLINE Transform(const Quaternion& q, 
		const Vector3& c = Vector3(tf2Scalar(0), tf2Scalar(0), tf2Scalar(0))) 
		: m_basis(q),
		m_origin(c)
	{}

  /**@brief Constructor from Matrix3x3 (optional Vector3)
   * @param b Rotation from Matrix 
   * @param c Translation from Vector default (0,0,0)*/
	explicit TF2SIMD_FORCE_INLINE Transform(const Matrix3x3& b, 
		const Vector3& c = Vector3(tf2Scalar(0), tf2Scalar(0), tf2Scalar(0)))
		: m_basis(b),
		m_origin(c)
	{}
  /**@brief Copy constructor */
	TF2SIMD_FORCE_INLINE Transform (const Transform& other)
		: m_basis(other.m_basis),
		m_origin(other.m_origin)
	{
	}
  /**@brief Assignment Operator */
	TF2SIMD_FORCE_INLINE Transform& operator=(const Transform& other)
	{
		m_basis = other.m_basis;
		m_origin = other.m_origin;
		return *this;
	}

  /**@brief Set the current transform as the value of the product of two transforms
   * @param t1 Transform 1
   * @param t2 Transform 2
   * This = Transform1 * Transform2 */
		TF2SIMD_FORCE_INLINE void mult(const Transform& t1, const Transform& t2) {
			m_basis = t1.m_basis * t2.m_basis;
			m_origin = t1(t2.m_origin);
		}

/*		void multInverseLeft(const Transform& t1, const Transform& t2) {
			Vector3 v = t2.m_origin - t1.m_origin;
			m_basis = tf2MultTransposeLeft(t1.m_basis, t2.m_basis);
			m_origin = v * t1.m_basis;
		}
		*/

/**@brief Return the transform of the vector */
	TF2SIMD_FORCE_INLINE Vector3 operator()(const Vector3& x) const
	{
		return Vector3(m_basis[0].dot(x) + m_origin.x(), 
			m_basis[1].dot(x) + m_origin.y(), 
			m_basis[2].dot(x) + m_origin.z());
	}

  /**@brief Return the transform of the vector */
	TF2SIMD_FORCE_INLINE Vector3 operator*(const Vector3& x) const
	{
		return (*this)(x);
	}

  /**@brief Return the transform of the Quaternion */
	TF2SIMD_FORCE_INLINE Quaternion operator*(const Quaternion& q) const
	{
		return getRotation() * q;
	}

  /**@brief Return the basis matrix for the rotation */
	TF2SIMD_FORCE_INLINE Matrix3x3&       getBasis()          { return m_basis; }
  /**@brief Return the basis matrix for the rotation */
	TF2SIMD_FORCE_INLINE const Matrix3x3& getBasis()    const { return m_basis; }

  /**@brief Return the origin vector translation */
	TF2SIMD_FORCE_INLINE Vector3&         getOrigin()         { return m_origin; }
  /**@brief Return the origin vector translation */
	TF2SIMD_FORCE_INLINE const Vector3&   getOrigin()   const { return m_origin; }

  /**@brief Return a quaternion representing the rotation */
	Quaternion getRotation() const { 
		Quaternion q;
		m_basis.getRotation(q);
		return q;
	}
	
	
  /**@brief Set from an array 
   * @param m A pointer to a 15 element array (12 rotation(row major padded on the right by 1), and 3 translation */
	void setFromOpenGLMatrix(const tf2Scalar *m)
	{
		m_basis.setFromOpenGLSubMatrix(m);
		m_origin.setValue(m[12],m[13],m[14]);
	}

  /**@brief Fill an array representation
   * @param m A pointer to a 15 element array (12 rotation(row major padded on the right by 1), and 3 translation */
	void getOpenGLMatrix(tf2Scalar *m) const 
	{
		m_basis.getOpenGLSubMatrix(m);
		m[12] = m_origin.x();
		m[13] = m_origin.y();
		m[14] = m_origin.z();
		m[15] = tf2Scalar(1.0);
	}

  /**@brief Set the translational element
   * @param origin The vector to set the translation to */
	TF2SIMD_FORCE_INLINE void setOrigin(const Vector3& origin) 
	{ 
		m_origin = origin;
	}

	TF2SIMD_FORCE_INLINE Vector3 invXform(const Vector3& inVec) const;


  /**@brief Set the rotational element by Matrix3x3 */
	TF2SIMD_FORCE_INLINE void setBasis(const Matrix3x3& basis)
	{ 
		m_basis = basis;
	}

  /**@brief Set the rotational element by Quaternion */
	TF2SIMD_FORCE_INLINE void setRotation(const Quaternion& q)
	{
		m_basis.setRotation(q);
	}


  /**@brief Set this transformation to the identity */
	void setIdentity()
	{
		m_basis.setIdentity();
		m_origin.setValue(tf2Scalar(0.0), tf2Scalar(0.0), tf2Scalar(0.0));
	}

  /**@brief Multiply this Transform by another(this = this * another) 
   * @param t The other transform */
	Transform& operator*=(const Transform& t) 
	{
		m_origin += m_basis * t.m_origin;
		m_basis *= t.m_basis;
		return *this;
	}

  /**@brief Return the inverse of this transform */
	Transform inverse() const
	{ 
		Matrix3x3 inv = m_basis.transpose();
		return Transform(inv, inv * -m_origin);
	}

  /**@brief Return the inverse of this transform times the other transform
   * @param t The other transform 
   * return this.inverse() * the other */
	Transform inverseTimes(const Transform& t) const;  

  /**@brief Return the product of this transform and the other */
	Transform operator*(const Transform& t) const;

  /**@brief Return an identity transform */
	static const Transform&	getIdentity()
	{
		static const Transform identityTransform(Matrix3x3::getIdentity());
		return identityTransform;
	}

	void	serialize(struct	TransformData& dataOut) const;

	void	serializeFloat(struct	TransformFloatData& dataOut) const;

	void	deSerialize(const struct	TransformData& dataIn);

	void	deSerializeDouble(const struct	TransformDoubleData& dataIn);

	void	deSerializeFloat(const struct	TransformFloatData& dataIn);

};


TF2SIMD_FORCE_INLINE Vector3
Transform::invXform(const Vector3& inVec) const
{
	Vector3 v = inVec - m_origin;
	return (m_basis.transpose() * v);
}

TF2SIMD_FORCE_INLINE Transform 
Transform::inverseTimes(const Transform& t) const  
{
	Vector3 v = t.getOrigin() - m_origin;
		return Transform(m_basis.transposeTimes(t.m_basis),
			v * m_basis);
}

TF2SIMD_FORCE_INLINE Transform 
Transform::operator*(const Transform& t) const
{
	return Transform(m_basis * t.m_basis, 
		(*this)(t.m_origin));
}

/**@brief Test if two transforms have all elements equal */
TF2SIMD_FORCE_INLINE bool operator==(const Transform& t1, const Transform& t2)
{
   return ( t1.getBasis()  == t2.getBasis() &&
            t1.getOrigin() == t2.getOrigin() );
}


///for serialization
struct	TransformFloatData
{
	Matrix3x3FloatData	m_basis;
	Vector3FloatData	m_origin;
};

struct	TransformDoubleData
{
	Matrix3x3DoubleData	m_basis;
	Vector3DoubleData	m_origin;
};



TF2SIMD_FORCE_INLINE	void	Transform::serialize(TransformData& dataOut) const
{
	m_basis.serialize(dataOut.m_basis);
	m_origin.serialize(dataOut.m_origin);
}

TF2SIMD_FORCE_INLINE	void	Transform::serializeFloat(TransformFloatData& dataOut) const
{
	m_basis.serializeFloat(dataOut.m_basis);
	m_origin.serializeFloat(dataOut.m_origin);
}


TF2SIMD_FORCE_INLINE	void	Transform::deSerialize(const TransformData& dataIn)
{
	m_basis.deSerialize(dataIn.m_basis);
	m_origin.deSerialize(dataIn.m_origin);
}

TF2SIMD_FORCE_INLINE	void	Transform::deSerializeFloat(const TransformFloatData& dataIn)
{
	m_basis.deSerializeFloat(dataIn.m_basis);
	m_origin.deSerializeFloat(dataIn.m_origin);
}

TF2SIMD_FORCE_INLINE	void	Transform::deSerializeDouble(const TransformDoubleData& dataIn)
{
	m_basis.deSerializeDouble(dataIn.m_basis);
	m_origin.deSerializeDouble(dataIn.m_origin);
}

}

#endif






