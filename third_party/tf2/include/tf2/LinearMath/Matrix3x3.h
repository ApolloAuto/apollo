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


#ifndef	TF2_MATRIX3x3_H
#define TF2_MATRIX3x3_H

#include "Vector3.h"
#include "Quaternion.h"

namespace tf2
{


#define Matrix3x3Data	Matrix3x3DoubleData 


/**@brief The Matrix3x3 class implements a 3x3 rotation matrix, to perform linear algebra in combination with Quaternion, Transform and Vector3.
* Make sure to only include a pure orthogonal matrix without scaling. */
class Matrix3x3 {

	///Data storage for the matrix, each vector is a row of the matrix
	Vector3 m_el[3];

public:
	/** @brief No initializaion constructor */
	Matrix3x3 () {}

	//		explicit Matrix3x3(const tf2Scalar *m) { setFromOpenGLSubMatrix(m); }

	/**@brief Constructor from Quaternion */
	explicit Matrix3x3(const Quaternion& q) { setRotation(q); }
	/*
	template <typename tf2Scalar>
	Matrix3x3(const tf2Scalar& yaw, const tf2Scalar& pitch, const tf2Scalar& roll)
	{ 
	setEulerYPR(yaw, pitch, roll);
	}
	*/
	/** @brief Constructor with row major formatting */
	Matrix3x3(const tf2Scalar& xx, const tf2Scalar& xy, const tf2Scalar& xz,
		const tf2Scalar& yx, const tf2Scalar& yy, const tf2Scalar& yz,
		const tf2Scalar& zx, const tf2Scalar& zy, const tf2Scalar& zz)
	{ 
		setValue(xx, xy, xz, 
			yx, yy, yz, 
			zx, zy, zz);
	}
	/** @brief Copy constructor */
	TF2SIMD_FORCE_INLINE Matrix3x3 (const Matrix3x3& other)
	{
		m_el[0] = other.m_el[0];
		m_el[1] = other.m_el[1];
		m_el[2] = other.m_el[2];
	}


	/** @brief Assignment Operator */
	TF2SIMD_FORCE_INLINE Matrix3x3& operator=(const Matrix3x3& other)
	{
		m_el[0] = other.m_el[0];
		m_el[1] = other.m_el[1];
		m_el[2] = other.m_el[2];
		return *this;
	}


	/** @brief Get a column of the matrix as a vector 
	*  @param i Column number 0 indexed */
	TF2SIMD_FORCE_INLINE Vector3 getColumn(int i) const
	{
		return Vector3(m_el[0][i],m_el[1][i],m_el[2][i]);
	}


	/** @brief Get a row of the matrix as a vector 
	*  @param i Row number 0 indexed */
	TF2SIMD_FORCE_INLINE const Vector3& getRow(int i) const
	{
		tf2FullAssert(0 <= i && i < 3);
		return m_el[i];
	}

	/** @brief Get a mutable reference to a row of the matrix as a vector 
	*  @param i Row number 0 indexed */
	TF2SIMD_FORCE_INLINE Vector3&  operator[](int i)
	{ 
		tf2FullAssert(0 <= i && i < 3);
		return m_el[i]; 
	}

	/** @brief Get a const reference to a row of the matrix as a vector 
	*  @param i Row number 0 indexed */
	TF2SIMD_FORCE_INLINE const Vector3& operator[](int i) const
	{
		tf2FullAssert(0 <= i && i < 3);
		return m_el[i]; 
	}

	/** @brief Multiply by the target matrix on the right
	*  @param m Rotation matrix to be applied 
	* Equivilant to this = this * m */
	Matrix3x3& operator*=(const Matrix3x3& m); 

	/** @brief Set from a carray of tf2Scalars 
	*  @param m A pointer to the beginning of an array of 9 tf2Scalars */
	void setFromOpenGLSubMatrix(const tf2Scalar *m)
	{
		m_el[0].setValue(m[0],m[4],m[8]);
		m_el[1].setValue(m[1],m[5],m[9]);
		m_el[2].setValue(m[2],m[6],m[10]);

	}
	/** @brief Set the values of the matrix explicitly (row major)
	*  @param xx Top left
	*  @param xy Top Middle
	*  @param xz Top Right
	*  @param yx Middle Left
	*  @param yy Middle Middle
	*  @param yz Middle Right
	*  @param zx Bottom Left
	*  @param zy Bottom Middle
	*  @param zz Bottom Right*/
	void setValue(const tf2Scalar& xx, const tf2Scalar& xy, const tf2Scalar& xz, 
		const tf2Scalar& yx, const tf2Scalar& yy, const tf2Scalar& yz, 
		const tf2Scalar& zx, const tf2Scalar& zy, const tf2Scalar& zz)
	{
		m_el[0].setValue(xx,xy,xz);
		m_el[1].setValue(yx,yy,yz);
		m_el[2].setValue(zx,zy,zz);
	}

	/** @brief Set the matrix from a quaternion
	*  @param q The Quaternion to match */  
	void setRotation(const Quaternion& q) 
	{
		tf2Scalar d = q.length2();
		tf2FullAssert(d != tf2Scalar(0.0));
		tf2Scalar s = tf2Scalar(2.0) / d;
		tf2Scalar xs = q.x() * s,   ys = q.y() * s,   zs = q.z() * s;
		tf2Scalar wx = q.w() * xs,  wy = q.w() * ys,  wz = q.w() * zs;
		tf2Scalar xx = q.x() * xs,  xy = q.x() * ys,  xz = q.x() * zs;
		tf2Scalar yy = q.y() * ys,  yz = q.y() * zs,  zz = q.z() * zs;
		setValue(tf2Scalar(1.0) - (yy + zz), xy - wz, xz + wy,
			xy + wz, tf2Scalar(1.0) - (xx + zz), yz - wx,
			xz - wy, yz + wx, tf2Scalar(1.0) - (xx + yy));
	}


	/** @brief Set the matrix from euler angles using YPR around ZYX respectively
	*  @param yaw Yaw about Z axis
	*  @param pitch Pitch about Y axis
	*  @param roll Roll about X axis 
	*/
	void setEulerZYX(const tf2Scalar& yaw, const tf2Scalar& pitch, const tf2Scalar& roll) __attribute__((deprecated))
	{
		setEulerYPR(yaw, pitch, roll);
	}

	/** @brief Set the matrix from euler angles YPR around ZYX axes
	* @param eulerZ Yaw aboud Z axis
	* @param eulerY Pitch around Y axis
	* @param eulerX Roll about X axis
	* 
	* These angles are used to produce a rotation matrix. The euler
	* angles are applied in ZYX order. I.e a vector is first rotated 
	* about X then Y and then Z
	**/
	void setEulerYPR(tf2Scalar eulerZ, tf2Scalar eulerY,tf2Scalar eulerX)  { 
		tf2Scalar ci ( tf2Cos(eulerX)); 
		tf2Scalar cj ( tf2Cos(eulerY)); 
		tf2Scalar ch ( tf2Cos(eulerZ)); 
		tf2Scalar si ( tf2Sin(eulerX)); 
		tf2Scalar sj ( tf2Sin(eulerY)); 
		tf2Scalar sh ( tf2Sin(eulerZ)); 
		tf2Scalar cc = ci * ch; 
		tf2Scalar cs = ci * sh; 
		tf2Scalar sc = si * ch; 
		tf2Scalar ss = si * sh;

		setValue(cj * ch, sj * sc - cs, sj * cc + ss,
			cj * sh, sj * ss + cc, sj * cs - sc, 
			-sj,      cj * si,      cj * ci);
	}

	/** @brief Set the matrix using RPY about XYZ fixed axes
	 * @param roll Roll about X axis
         * @param pitch Pitch around Y axis
         * @param yaw Yaw aboud Z axis
         * 
	 **/
	void setRPY(tf2Scalar roll, tf2Scalar pitch,tf2Scalar yaw) { 
               setEulerYPR(yaw, pitch, roll);
	}

	/**@brief Set the matrix to the identity */
	void setIdentity()
	{ 
		setValue(tf2Scalar(1.0), tf2Scalar(0.0), tf2Scalar(0.0), 
			tf2Scalar(0.0), tf2Scalar(1.0), tf2Scalar(0.0), 
			tf2Scalar(0.0), tf2Scalar(0.0), tf2Scalar(1.0)); 
	}

	static const Matrix3x3&	getIdentity()
	{
		static const Matrix3x3 identityMatrix(tf2Scalar(1.0), tf2Scalar(0.0), tf2Scalar(0.0), 
			tf2Scalar(0.0), tf2Scalar(1.0), tf2Scalar(0.0), 
			tf2Scalar(0.0), tf2Scalar(0.0), tf2Scalar(1.0));
		return identityMatrix;
	}

	/**@brief Fill the values of the matrix into a 9 element array 
	* @param m The array to be filled */
	void getOpenGLSubMatrix(tf2Scalar *m) const 
	{
		m[0]  = tf2Scalar(m_el[0].x()); 
		m[1]  = tf2Scalar(m_el[1].x());
		m[2]  = tf2Scalar(m_el[2].x());
		m[3]  = tf2Scalar(0.0); 
		m[4]  = tf2Scalar(m_el[0].y());
		m[5]  = tf2Scalar(m_el[1].y());
		m[6]  = tf2Scalar(m_el[2].y());
		m[7]  = tf2Scalar(0.0); 
		m[8]  = tf2Scalar(m_el[0].z()); 
		m[9]  = tf2Scalar(m_el[1].z());
		m[10] = tf2Scalar(m_el[2].z());
		m[11] = tf2Scalar(0.0); 
	}

	/**@brief Get the matrix represented as a quaternion 
	* @param q The quaternion which will be set */
	void getRotation(Quaternion& q) const
	{
		tf2Scalar trace = m_el[0].x() + m_el[1].y() + m_el[2].z();
		tf2Scalar temp[4];

		if (trace > tf2Scalar(0.0)) 
		{
			tf2Scalar s = tf2Sqrt(trace + tf2Scalar(1.0));
			temp[3]=(s * tf2Scalar(0.5));
			s = tf2Scalar(0.5) / s;

			temp[0]=((m_el[2].y() - m_el[1].z()) * s);
			temp[1]=((m_el[0].z() - m_el[2].x()) * s);
			temp[2]=((m_el[1].x() - m_el[0].y()) * s);
		} 
		else 
		{
			int i = m_el[0].x() < m_el[1].y() ? 
				(m_el[1].y() < m_el[2].z() ? 2 : 1) :
				(m_el[0].x() < m_el[2].z() ? 2 : 0); 
			int j = (i + 1) % 3;  
			int k = (i + 2) % 3;

			tf2Scalar s = tf2Sqrt(m_el[i][i] - m_el[j][j] - m_el[k][k] + tf2Scalar(1.0));
			temp[i] = s * tf2Scalar(0.5);
			s = tf2Scalar(0.5) / s;

			temp[3] = (m_el[k][j] - m_el[j][k]) * s;
			temp[j] = (m_el[j][i] + m_el[i][j]) * s;
			temp[k] = (m_el[k][i] + m_el[i][k]) * s;
		}
		q.setValue(temp[0],temp[1],temp[2],temp[3]);
	}

	/**@brief Get the matrix represented as euler angles around ZYX
	* @param yaw Yaw around Z axis
	* @param pitch Pitch around Y axis
	* @param roll around X axis 
 	* @param solution_number Which solution of two possible solutions ( 1 or 2) are possible values*/	
	__attribute__((deprecated)) void getEulerZYX(tf2Scalar& yaw, tf2Scalar& pitch, tf2Scalar& roll, unsigned int solution_number = 1) const
	{
		getEulerYPR(yaw, pitch, roll, solution_number);
	};


	/**@brief Get the matrix represented as euler angles around YXZ, roundtrip with setEulerYPR
	* @param yaw Yaw around Z axis
	* @param pitch Pitch around Y axis
	* @param roll around X axis */	
	void getEulerYPR(tf2Scalar& yaw, tf2Scalar& pitch, tf2Scalar& roll, unsigned int solution_number = 1) const
	{
		struct Euler
		{
			tf2Scalar yaw;
			tf2Scalar pitch;
			tf2Scalar roll;
		};

		Euler euler_out;
		Euler euler_out2; //second solution
		//get the pointer to the raw data

		// Check that pitch is not at a singularity
  		// Check that pitch is not at a singularity
		if (tf2Fabs(m_el[2].x()) >= 1)
		{
			euler_out.yaw = 0;
			euler_out2.yaw = 0;
	
			// From difference of angles formula
			tf2Scalar delta = tf2Atan2(m_el[2].y(),m_el[2].z());
			if (m_el[2].x() < 0)  //gimbal locked down
			{
				euler_out.pitch = TF2SIMD_PI / tf2Scalar(2.0);
				euler_out2.pitch = TF2SIMD_PI / tf2Scalar(2.0);
				euler_out.roll = delta;
				euler_out2.roll = delta;
			}
			else // gimbal locked up
			{
				euler_out.pitch = -TF2SIMD_PI / tf2Scalar(2.0);
				euler_out2.pitch = -TF2SIMD_PI / tf2Scalar(2.0);
				euler_out.roll = delta;
				euler_out2.roll = delta;
			}
		}
		else
		{
			euler_out.pitch = - tf2Asin(m_el[2].x());
			euler_out2.pitch = TF2SIMD_PI - euler_out.pitch;

			euler_out.roll = tf2Atan2(m_el[2].y()/tf2Cos(euler_out.pitch), 
				m_el[2].z()/tf2Cos(euler_out.pitch));
			euler_out2.roll = tf2Atan2(m_el[2].y()/tf2Cos(euler_out2.pitch), 
				m_el[2].z()/tf2Cos(euler_out2.pitch));

			euler_out.yaw = tf2Atan2(m_el[1].x()/tf2Cos(euler_out.pitch), 
				m_el[0].x()/tf2Cos(euler_out.pitch));
			euler_out2.yaw = tf2Atan2(m_el[1].x()/tf2Cos(euler_out2.pitch), 
				m_el[0].x()/tf2Cos(euler_out2.pitch));
		}

		if (solution_number == 1)
		{ 
			yaw = euler_out.yaw; 
			pitch = euler_out.pitch;
			roll = euler_out.roll;
		}
		else
		{ 
			yaw = euler_out2.yaw; 
			pitch = euler_out2.pitch;
			roll = euler_out2.roll;
		}
	}

	/**@brief Get the matrix represented as roll pitch and yaw about fixed axes XYZ
	* @param roll around X axis 
	* @param pitch Pitch around Y axis
	* @param yaw Yaw around Z axis
	* @param solution_number Which solution of two possible solutions ( 1 or 2) are possible values*/	
	void getRPY(tf2Scalar& roll, tf2Scalar& pitch, tf2Scalar& yaw, unsigned int solution_number = 1) const
	{
	getEulerYPR(yaw, pitch, roll, solution_number);
	}

	/**@brief Create a scaled copy of the matrix 
	* @param s Scaling vector The elements of the vector will scale each column */

	Matrix3x3 scaled(const Vector3& s) const
	{
		return Matrix3x3(m_el[0].x() * s.x(), m_el[0].y() * s.y(), m_el[0].z() * s.z(),
			m_el[1].x() * s.x(), m_el[1].y() * s.y(), m_el[1].z() * s.z(),
			m_el[2].x() * s.x(), m_el[2].y() * s.y(), m_el[2].z() * s.z());
	}

	/**@brief Return the determinant of the matrix */
	tf2Scalar            determinant() const;
	/**@brief Return the adjoint of the matrix */
	Matrix3x3 adjoint() const;
	/**@brief Return the matrix with all values non negative */
	Matrix3x3 absolute() const;
	/**@brief Return the transpose of the matrix */
	Matrix3x3 transpose() const;
	/**@brief Return the inverse of the matrix */
	Matrix3x3 inverse() const; 

	Matrix3x3 transposeTimes(const Matrix3x3& m) const;
	Matrix3x3 timesTranspose(const Matrix3x3& m) const;

	TF2SIMD_FORCE_INLINE tf2Scalar tdotx(const Vector3& v) const 
	{
		return m_el[0].x() * v.x() + m_el[1].x() * v.y() + m_el[2].x() * v.z();
	}
	TF2SIMD_FORCE_INLINE tf2Scalar tdoty(const Vector3& v) const 
	{
		return m_el[0].y() * v.x() + m_el[1].y() * v.y() + m_el[2].y() * v.z();
	}
	TF2SIMD_FORCE_INLINE tf2Scalar tdotz(const Vector3& v) const 
	{
		return m_el[0].z() * v.x() + m_el[1].z() * v.y() + m_el[2].z() * v.z();
	}


	/**@brief diagonalizes this matrix by the Jacobi method.
	* @param rot stores the rotation from the coordinate system in which the matrix is diagonal to the original
	* coordinate system, i.e., old_this = rot * new_this * rot^T. 
	* @param threshold See iteration
	* @param iteration The iteration stops when all off-diagonal elements are less than the threshold multiplied 
	* by the sum of the absolute values of the diagonal, or when maxSteps have been executed. 
	* 
	* Note that this matrix is assumed to be symmetric. 
	*/
	void diagonalize(Matrix3x3& rot, tf2Scalar threshold, int maxSteps)
	{
		rot.setIdentity();
		for (int step = maxSteps; step > 0; step--)
		{
			// find off-diagonal element [p][q] with largest magnitude
			int p = 0;
			int q = 1;
			int r = 2;
			tf2Scalar max = tf2Fabs(m_el[0][1]);
			tf2Scalar v = tf2Fabs(m_el[0][2]);
			if (v > max)
			{
				q = 2;
				r = 1;
				max = v;
			}
			v = tf2Fabs(m_el[1][2]);
			if (v > max)
			{
				p = 1;
				q = 2;
				r = 0;
				max = v;
			}

			tf2Scalar t = threshold * (tf2Fabs(m_el[0][0]) + tf2Fabs(m_el[1][1]) + tf2Fabs(m_el[2][2]));
			if (max <= t)
			{
				if (max <= TF2SIMD_EPSILON * t)
				{
					return;
				}
				step = 1;
			}

			// compute Jacobi rotation J which leads to a zero for element [p][q] 
			tf2Scalar mpq = m_el[p][q];
			tf2Scalar theta = (m_el[q][q] - m_el[p][p]) / (2 * mpq);
			tf2Scalar theta2 = theta * theta;
			tf2Scalar cos;
			tf2Scalar sin;
			if (theta2 * theta2 < tf2Scalar(10 / TF2SIMD_EPSILON))
			{
				t = (theta >= 0) ? 1 / (theta + tf2Sqrt(1 + theta2))
					: 1 / (theta - tf2Sqrt(1 + theta2));
				cos = 1 / tf2Sqrt(1 + t * t);
				sin = cos * t;
			}
			else
			{
				// approximation for large theta-value, i.e., a nearly diagonal matrix
				t = 1 / (theta * (2 + tf2Scalar(0.5) / theta2));
				cos = 1 - tf2Scalar(0.5) * t * t;
				sin = cos * t;
			}

			// apply rotation to matrix (this = J^T * this * J)
			m_el[p][q] = m_el[q][p] = 0;
			m_el[p][p] -= t * mpq;
			m_el[q][q] += t * mpq;
			tf2Scalar mrp = m_el[r][p];
			tf2Scalar mrq = m_el[r][q];
			m_el[r][p] = m_el[p][r] = cos * mrp - sin * mrq;
			m_el[r][q] = m_el[q][r] = cos * mrq + sin * mrp;

			// apply rotation to rot (rot = rot * J)
			for (int i = 0; i < 3; i++)
			{
				Vector3& row = rot[i];
				mrp = row[p];
				mrq = row[q];
				row[p] = cos * mrp - sin * mrq;
				row[q] = cos * mrq + sin * mrp;
			}
		}
	}




	/**@brief Calculate the matrix cofactor 
	* @param r1 The first row to use for calculating the cofactor
	* @param c1 The first column to use for calculating the cofactor
	* @param r1 The second row to use for calculating the cofactor
	* @param c1 The second column to use for calculating the cofactor
	* See http://en.wikipedia.org/wiki/Cofactor_(linear_algebra) for more details
	*/
	tf2Scalar cofac(int r1, int c1, int r2, int c2) const 
	{
		return m_el[r1][c1] * m_el[r2][c2] - m_el[r1][c2] * m_el[r2][c1];
	}

	void	serialize(struct	Matrix3x3Data& dataOut) const;

	void	serializeFloat(struct	Matrix3x3FloatData& dataOut) const;

	void	deSerialize(const struct	Matrix3x3Data& dataIn);

	void	deSerializeFloat(const struct	Matrix3x3FloatData& dataIn);

	void	deSerializeDouble(const struct	Matrix3x3DoubleData& dataIn);

};


TF2SIMD_FORCE_INLINE Matrix3x3& 
Matrix3x3::operator*=(const Matrix3x3& m)
{
	setValue(m.tdotx(m_el[0]), m.tdoty(m_el[0]), m.tdotz(m_el[0]),
		m.tdotx(m_el[1]), m.tdoty(m_el[1]), m.tdotz(m_el[1]),
		m.tdotx(m_el[2]), m.tdoty(m_el[2]), m.tdotz(m_el[2]));
	return *this;
}

TF2SIMD_FORCE_INLINE tf2Scalar 
Matrix3x3::determinant() const
{ 
	return tf2Triple((*this)[0], (*this)[1], (*this)[2]);
}


TF2SIMD_FORCE_INLINE Matrix3x3 
Matrix3x3::absolute() const
{
	return Matrix3x3(
		tf2Fabs(m_el[0].x()), tf2Fabs(m_el[0].y()), tf2Fabs(m_el[0].z()),
		tf2Fabs(m_el[1].x()), tf2Fabs(m_el[1].y()), tf2Fabs(m_el[1].z()),
		tf2Fabs(m_el[2].x()), tf2Fabs(m_el[2].y()), tf2Fabs(m_el[2].z()));
}

TF2SIMD_FORCE_INLINE Matrix3x3 
Matrix3x3::transpose() const 
{
	return Matrix3x3(m_el[0].x(), m_el[1].x(), m_el[2].x(),
		m_el[0].y(), m_el[1].y(), m_el[2].y(),
		m_el[0].z(), m_el[1].z(), m_el[2].z());
}

TF2SIMD_FORCE_INLINE Matrix3x3 
Matrix3x3::adjoint() const 
{
	return Matrix3x3(cofac(1, 1, 2, 2), cofac(0, 2, 2, 1), cofac(0, 1, 1, 2),
		cofac(1, 2, 2, 0), cofac(0, 0, 2, 2), cofac(0, 2, 1, 0),
		cofac(1, 0, 2, 1), cofac(0, 1, 2, 0), cofac(0, 0, 1, 1));
}

TF2SIMD_FORCE_INLINE Matrix3x3 
Matrix3x3::inverse() const
{
	Vector3 co(cofac(1, 1, 2, 2), cofac(1, 2, 2, 0), cofac(1, 0, 2, 1));
	tf2Scalar det = (*this)[0].dot(co);
	tf2FullAssert(det != tf2Scalar(0.0));
	tf2Scalar s = tf2Scalar(1.0) / det;
	return Matrix3x3(co.x() * s, cofac(0, 2, 2, 1) * s, cofac(0, 1, 1, 2) * s,
		co.y() * s, cofac(0, 0, 2, 2) * s, cofac(0, 2, 1, 0) * s,
		co.z() * s, cofac(0, 1, 2, 0) * s, cofac(0, 0, 1, 1) * s);
}

TF2SIMD_FORCE_INLINE Matrix3x3 
Matrix3x3::transposeTimes(const Matrix3x3& m) const
{
	return Matrix3x3(
		m_el[0].x() * m[0].x() + m_el[1].x() * m[1].x() + m_el[2].x() * m[2].x(),
		m_el[0].x() * m[0].y() + m_el[1].x() * m[1].y() + m_el[2].x() * m[2].y(),
		m_el[0].x() * m[0].z() + m_el[1].x() * m[1].z() + m_el[2].x() * m[2].z(),
		m_el[0].y() * m[0].x() + m_el[1].y() * m[1].x() + m_el[2].y() * m[2].x(),
		m_el[0].y() * m[0].y() + m_el[1].y() * m[1].y() + m_el[2].y() * m[2].y(),
		m_el[0].y() * m[0].z() + m_el[1].y() * m[1].z() + m_el[2].y() * m[2].z(),
		m_el[0].z() * m[0].x() + m_el[1].z() * m[1].x() + m_el[2].z() * m[2].x(),
		m_el[0].z() * m[0].y() + m_el[1].z() * m[1].y() + m_el[2].z() * m[2].y(),
		m_el[0].z() * m[0].z() + m_el[1].z() * m[1].z() + m_el[2].z() * m[2].z());
}

TF2SIMD_FORCE_INLINE Matrix3x3 
Matrix3x3::timesTranspose(const Matrix3x3& m) const
{
	return Matrix3x3(
		m_el[0].dot(m[0]), m_el[0].dot(m[1]), m_el[0].dot(m[2]),
		m_el[1].dot(m[0]), m_el[1].dot(m[1]), m_el[1].dot(m[2]),
		m_el[2].dot(m[0]), m_el[2].dot(m[1]), m_el[2].dot(m[2]));

}

TF2SIMD_FORCE_INLINE Vector3 
operator*(const Matrix3x3& m, const Vector3& v) 
{
	return Vector3(m[0].dot(v), m[1].dot(v), m[2].dot(v));
}


TF2SIMD_FORCE_INLINE Vector3
operator*(const Vector3& v, const Matrix3x3& m)
{
	return Vector3(m.tdotx(v), m.tdoty(v), m.tdotz(v));
}

TF2SIMD_FORCE_INLINE Matrix3x3 
operator*(const Matrix3x3& m1, const Matrix3x3& m2)
{
	return Matrix3x3(
		m2.tdotx( m1[0]), m2.tdoty( m1[0]), m2.tdotz( m1[0]),
		m2.tdotx( m1[1]), m2.tdoty( m1[1]), m2.tdotz( m1[1]),
		m2.tdotx( m1[2]), m2.tdoty( m1[2]), m2.tdotz( m1[2]));
}

/*
TF2SIMD_FORCE_INLINE Matrix3x3 tf2MultTransposeLeft(const Matrix3x3& m1, const Matrix3x3& m2) {
return Matrix3x3(
m1[0][0] * m2[0][0] + m1[1][0] * m2[1][0] + m1[2][0] * m2[2][0],
m1[0][0] * m2[0][1] + m1[1][0] * m2[1][1] + m1[2][0] * m2[2][1],
m1[0][0] * m2[0][2] + m1[1][0] * m2[1][2] + m1[2][0] * m2[2][2],
m1[0][1] * m2[0][0] + m1[1][1] * m2[1][0] + m1[2][1] * m2[2][0],
m1[0][1] * m2[0][1] + m1[1][1] * m2[1][1] + m1[2][1] * m2[2][1],
m1[0][1] * m2[0][2] + m1[1][1] * m2[1][2] + m1[2][1] * m2[2][2],
m1[0][2] * m2[0][0] + m1[1][2] * m2[1][0] + m1[2][2] * m2[2][0],
m1[0][2] * m2[0][1] + m1[1][2] * m2[1][1] + m1[2][2] * m2[2][1],
m1[0][2] * m2[0][2] + m1[1][2] * m2[1][2] + m1[2][2] * m2[2][2]);
}
*/

/**@brief Equality operator between two matrices
* It will test all elements are equal.  */
TF2SIMD_FORCE_INLINE bool operator==(const Matrix3x3& m1, const Matrix3x3& m2)
{
	return ( m1[0][0] == m2[0][0] && m1[1][0] == m2[1][0] && m1[2][0] == m2[2][0] &&
		m1[0][1] == m2[0][1] && m1[1][1] == m2[1][1] && m1[2][1] == m2[2][1] &&
		m1[0][2] == m2[0][2] && m1[1][2] == m2[1][2] && m1[2][2] == m2[2][2] );
}

///for serialization
struct	Matrix3x3FloatData
{
	Vector3FloatData m_el[3];
};

///for serialization
struct	Matrix3x3DoubleData
{
	Vector3DoubleData m_el[3];
};


	

TF2SIMD_FORCE_INLINE	void	Matrix3x3::serialize(struct	Matrix3x3Data& dataOut) const
{
	for (int i=0;i<3;i++)
		m_el[i].serialize(dataOut.m_el[i]);
}

TF2SIMD_FORCE_INLINE	void	Matrix3x3::serializeFloat(struct	Matrix3x3FloatData& dataOut) const
{
	for (int i=0;i<3;i++)
		m_el[i].serializeFloat(dataOut.m_el[i]);
}


TF2SIMD_FORCE_INLINE	void	Matrix3x3::deSerialize(const struct	Matrix3x3Data& dataIn)
{
	for (int i=0;i<3;i++)
		m_el[i].deSerialize(dataIn.m_el[i]);
}

TF2SIMD_FORCE_INLINE	void	Matrix3x3::deSerializeFloat(const struct	Matrix3x3FloatData& dataIn)
{
	for (int i=0;i<3;i++)
		m_el[i].deSerializeFloat(dataIn.m_el[i]);
}

TF2SIMD_FORCE_INLINE	void	Matrix3x3::deSerializeDouble(const struct	Matrix3x3DoubleData& dataIn)
{
	for (int i=0;i<3;i++)
		m_el[i].deSerializeDouble(dataIn.m_el[i]);
}

}
#endif //TF2_MATRIX3x3_H

