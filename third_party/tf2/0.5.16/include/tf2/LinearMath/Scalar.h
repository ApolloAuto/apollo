/*
Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef TF2_SCALAR_H
#define TF2_SCALAR_H

#ifdef TF2_MANAGED_CODE
//Aligned data types not supported in managed code
#pragma unmanaged
#endif


#include <math.h>
#include <stdlib.h>//size_t for MSVC 6.0
#include <cstdlib>
#include <cfloat>
#include <float.h>

#if defined(DEBUG) || defined (_DEBUG)
#define TF2_DEBUG
#endif


#ifdef _WIN32

		#if defined(__MINGW32__) || defined(__CYGWIN__) || (defined (_MSC_VER) && _MSC_VER < 1300)

			#define TF2SIMD_FORCE_INLINE inline
			#define ATTRIBUTE_ALIGNED16(a) a
			#define ATTRIBUTE_ALIGNED64(a) a
			#define ATTRIBUTE_ALIGNED128(a) a
		#else
			//#define TF2_HAS_ALIGNED_ALLOCATOR
			#pragma warning(disable : 4324) // disable padding warning
//			#pragma warning(disable:4530) // Disable the exception disable but used in MSCV Stl warning.
//			#pragma warning(disable:4996) //Turn off warnings about deprecated C routines
//			#pragma warning(disable:4786) // Disable the "debug name too long" warning

			#define TF2SIMD_FORCE_INLINE __forceinline
			#define ATTRIBUTE_ALIGNED16(a) __declspec(align(16)) a
			#define ATTRIBUTE_ALIGNED64(a) __declspec(align(64)) a
			#define ATTRIBUTE_ALIGNED128(a) __declspec (align(128)) a
		#ifdef _XBOX
			#define TF2_USE_VMX128

			#include <ppcintrinsics.h>
 			#define TF2_HAVE_NATIVE_FSEL
 			#define tf2Fsel(a,b,c) __fsel((a),(b),(c))
		#else


		#endif//_XBOX

		#endif //__MINGW32__

		#include <assert.h>
#ifdef TF2_DEBUG
		#define tf2Assert assert
#else
		#define tf2Assert(x)
#endif
		//tf2FullAssert is optional, slows down a lot
		#define tf2FullAssert(x)

		#define tf2Likely(_c)  _c
		#define tf2Unlikely(_c) _c

#else
	
#if defined	(__CELLOS_LV2__)
		#define TF2SIMD_FORCE_INLINE inline
		#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		#define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
		#define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#ifndef assert
		#include <assert.h>
		#endif
#ifdef TF2_DEBUG
		#define tf2Assert assert
#else
		#define tf2Assert(x)
#endif
		//tf2FullAssert is optional, slows down a lot
		#define tf2FullAssert(x)

		#define tf2Likely(_c)  _c
		#define tf2Unlikely(_c) _c

#else

#ifdef USE_LIBSPE2

		#define TF2SIMD_FORCE_INLINE __inline
		#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		#define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
		#define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#ifndef assert
		#include <assert.h>
		#endif
#ifdef TF2_DEBUG
		#define tf2Assert assert
#else
		#define tf2Assert(x)
#endif
		//tf2FullAssert is optional, slows down a lot
		#define tf2FullAssert(x)


		#define tf2Likely(_c)   __builtin_expect((_c), 1)
		#define tf2Unlikely(_c) __builtin_expect((_c), 0)
		

#else
	//non-windows systems

		#define TF2SIMD_FORCE_INLINE inline
		///@todo: check out alignment methods for other platforms/compilers
		///#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		///#define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
		///#define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
		#define ATTRIBUTE_ALIGNED16(a) a
		#define ATTRIBUTE_ALIGNED64(a) a
		#define ATTRIBUTE_ALIGNED128(a) a
		#ifndef assert
		#include <assert.h>
		#endif

#if defined(DEBUG) || defined (_DEBUG)
		#define tf2Assert assert
#else
		#define tf2Assert(x)
#endif

		//tf2FullAssert is optional, slows down a lot
		#define tf2FullAssert(x)
		#define tf2Likely(_c)  _c
		#define tf2Unlikely(_c) _c

#endif // LIBSPE2

#endif	//__CELLOS_LV2__
#endif


///The tf2Scalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
typedef double tf2Scalar;
//this number could be bigger in double precision
#define TF2_LARGE_FLOAT 1e30


#define TF2_DECLARE_ALIGNED_ALLOCATOR() \
   TF2SIMD_FORCE_INLINE void* operator new(size_t sizeInBytes)   { return tf2AlignedAlloc(sizeInBytes,16); }   \
   TF2SIMD_FORCE_INLINE void  operator delete(void* ptr)         { tf2AlignedFree(ptr); }   \
   TF2SIMD_FORCE_INLINE void* operator new(size_t, void* ptr)   { return ptr; }   \
   TF2SIMD_FORCE_INLINE void  operator delete(void*, void*)      { }   \
   TF2SIMD_FORCE_INLINE void* operator new[](size_t sizeInBytes)   { return tf2AlignedAlloc(sizeInBytes,16); }   \
   TF2SIMD_FORCE_INLINE void  operator delete[](void* ptr)         { tf2AlignedFree(ptr); }   \
   TF2SIMD_FORCE_INLINE void* operator new[](size_t, void* ptr)   { return ptr; }   \
   TF2SIMD_FORCE_INLINE void  operator delete[](void*, void*)      { }   \



		
TF2SIMD_FORCE_INLINE tf2Scalar tf2Sqrt(tf2Scalar x) { return sqrt(x); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Fabs(tf2Scalar x) { return fabs(x); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Cos(tf2Scalar x) { return cos(x); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Sin(tf2Scalar x) { return sin(x); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Tan(tf2Scalar x) { return tan(x); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Acos(tf2Scalar x) { if (x<tf2Scalar(-1))	x=tf2Scalar(-1); if (x>tf2Scalar(1))	x=tf2Scalar(1); return acos(x); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Asin(tf2Scalar x) { if (x<tf2Scalar(-1))	x=tf2Scalar(-1); if (x>tf2Scalar(1))	x=tf2Scalar(1); return asin(x); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Atan(tf2Scalar x) { return atan(x); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Atan2(tf2Scalar x, tf2Scalar y) { return atan2(x, y); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Exp(tf2Scalar x) { return exp(x); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Log(tf2Scalar x) { return log(x); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Pow(tf2Scalar x,tf2Scalar y) { return pow(x,y); }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Fmod(tf2Scalar x,tf2Scalar y) { return fmod(x,y); }


#define TF2SIMD_2_PI         tf2Scalar(6.283185307179586232)
#define TF2SIMD_PI           (TF2SIMD_2_PI * tf2Scalar(0.5))
#define TF2SIMD_HALF_PI      (TF2SIMD_2_PI * tf2Scalar(0.25))
#define TF2SIMD_RADS_PER_DEG (TF2SIMD_2_PI / tf2Scalar(360.0))
#define TF2SIMD_DEGS_PER_RAD  (tf2Scalar(360.0) / TF2SIMD_2_PI)
#define TF2SIMDSQRT12 tf2Scalar(0.7071067811865475244008443621048490)

#define tf2RecipSqrt(x) ((tf2Scalar)(tf2Scalar(1.0)/tf2Sqrt(tf2Scalar(x))))		/* reciprocal square root */


#define TF2SIMD_EPSILON      DBL_EPSILON
#define TF2SIMD_INFINITY     DBL_MAX

TF2SIMD_FORCE_INLINE tf2Scalar tf2Atan2Fast(tf2Scalar y, tf2Scalar x) 
{
	tf2Scalar coeff_1 = TF2SIMD_PI / 4.0f;
	tf2Scalar coeff_2 = 3.0f * coeff_1;
	tf2Scalar abs_y = tf2Fabs(y);
	tf2Scalar angle;
	if (x >= 0.0f) {
		tf2Scalar r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	} else {
		tf2Scalar r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}
	return (y < 0.0f) ? -angle : angle;
}

TF2SIMD_FORCE_INLINE bool      tf2FuzzyZero(tf2Scalar x) { return tf2Fabs(x) < TF2SIMD_EPSILON; }

TF2SIMD_FORCE_INLINE bool	tf2Equal(tf2Scalar a, tf2Scalar eps) {
	return (((a) <= eps) && !((a) < -eps));
}
TF2SIMD_FORCE_INLINE bool	tf2GreaterEqual (tf2Scalar a, tf2Scalar eps) {
	return (!((a) <= eps));
}


TF2SIMD_FORCE_INLINE int       tf2IsNegative(tf2Scalar x) {
    return x < tf2Scalar(0.0) ? 1 : 0;
}

TF2SIMD_FORCE_INLINE tf2Scalar tf2Radians(tf2Scalar x) { return x * TF2SIMD_RADS_PER_DEG; }
TF2SIMD_FORCE_INLINE tf2Scalar tf2Degrees(tf2Scalar x) { return x * TF2SIMD_DEGS_PER_RAD; }

#define TF2_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

#ifndef tf2Fsel
TF2SIMD_FORCE_INLINE tf2Scalar tf2Fsel(tf2Scalar a, tf2Scalar b, tf2Scalar c)
{
	return a >= 0 ? b : c;
}
#endif
#define tf2Fsels(a,b,c) (tf2Scalar)tf2Fsel(a,b,c)


TF2SIMD_FORCE_INLINE bool tf2MachineIsLittleEndian()
{
   long int i = 1;
   const char *p = (const char *) &i;
   if (p[0] == 1)  // Lowest address contains the least significant byte
	   return true;
   else
	   return false;
}



///tf2Select avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
TF2SIMD_FORCE_INLINE unsigned tf2Select(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero) 
{
    // Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
    // Rely on positive value or'ed with its negative having sign bit on
    // and zero value or'ed with its negative (which is still zero) having sign bit off 
    // Use arithmetic shift right, shifting the sign bit through all 32 bits
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz;
    return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz)); 
}
TF2SIMD_FORCE_INLINE int tf2Select(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
{
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz; 
    return static_cast<int>((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}
TF2SIMD_FORCE_INLINE float tf2Select(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
{
#ifdef TF2_HAVE_NATIVE_FSEL
    return (float)tf2Fsel((tf2Scalar)condition - tf2Scalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
#else
    return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero; 
#endif
}

template<typename T> TF2SIMD_FORCE_INLINE void tf2Swap(T& a, T& b)
{
	T tmp = a;
	a = b;
	b = tmp;
}


//PCK: endian swapping functions
TF2SIMD_FORCE_INLINE unsigned tf2SwapEndian(unsigned val)
{
	return (((val & 0xff000000) >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8)  | ((val & 0x000000ff) << 24));
}

TF2SIMD_FORCE_INLINE unsigned short tf2SwapEndian(unsigned short val)
{
	return static_cast<unsigned short>(((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
}

TF2SIMD_FORCE_INLINE unsigned tf2SwapEndian(int val)
{
	return tf2SwapEndian((unsigned)val);
}

TF2SIMD_FORCE_INLINE unsigned short tf2SwapEndian(short val)
{
	return tf2SwapEndian((unsigned short) val);
}

///tf2SwapFloat uses using char pointers to swap the endianness
////tf2SwapFloat/tf2SwapDouble will NOT return a float, because the machine might 'correct' invalid floating point values
///Not all values of sign/exponent/mantissa are valid floating point numbers according to IEEE 754. 
///When a floating point unit is faced with an invalid value, it may actually change the value, or worse, throw an exception. 
///In most systems, running user mode code, you wouldn't get an exception, but instead the hardware/os/runtime will 'fix' the number for you. 
///so instead of returning a float/double, we return integer/long long integer
TF2SIMD_FORCE_INLINE unsigned int  tf2SwapEndianFloat(float d)
{
    unsigned int a = 0;
    unsigned char *dst = (unsigned char *)&a;
    unsigned char *src = (unsigned char *)&d;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];
    return a;
}

// unswap using char pointers
TF2SIMD_FORCE_INLINE float tf2UnswapEndianFloat(unsigned int a) 
{
    float d = 0.0f;
    unsigned char *src = (unsigned char *)&a;
    unsigned char *dst = (unsigned char *)&d;

    dst[0] = src[3];
    dst[1] = src[2];
    dst[2] = src[1];
    dst[3] = src[0];

    return d;
}


// swap using char pointers
TF2SIMD_FORCE_INLINE void  tf2SwapEndianDouble(double d, unsigned char* dst)
{
    unsigned char *src = (unsigned char *)&d;

    dst[0] = src[7];
    dst[1] = src[6];
    dst[2] = src[5];
    dst[3] = src[4];
    dst[4] = src[3];
    dst[5] = src[2];
    dst[6] = src[1];
    dst[7] = src[0];

}

// unswap using char pointers
TF2SIMD_FORCE_INLINE double tf2UnswapEndianDouble(const unsigned char *src) 
{
    double d = 0.0;
    unsigned char *dst = (unsigned char *)&d;

    dst[0] = src[7];
    dst[1] = src[6];
    dst[2] = src[5];
    dst[3] = src[4];
    dst[4] = src[3];
    dst[5] = src[2];
    dst[6] = src[1];
    dst[7] = src[0];

	return d;
}

// returns normalized value in range [-TF2SIMD_PI, TF2SIMD_PI]
TF2SIMD_FORCE_INLINE tf2Scalar tf2NormalizeAngle(tf2Scalar angleInRadians) 
{
	angleInRadians = tf2Fmod(angleInRadians, TF2SIMD_2_PI);
	if(angleInRadians < -TF2SIMD_PI)
	{
		return angleInRadians + TF2SIMD_2_PI;
	}
	else if(angleInRadians > TF2SIMD_PI)
	{
		return angleInRadians - TF2SIMD_2_PI;
	}
	else
	{
		return angleInRadians;
	}
}

///rudimentary class to provide type info
struct tf2TypedObject
{
	tf2TypedObject(int objectType)
		:m_objectType(objectType)
	{
	}
	int	m_objectType;
	inline int getObjectType() const
	{
		return m_objectType;
	}
};
#endif //TF2SIMD___SCALAR_H
