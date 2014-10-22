#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H
#include <memory.h>
#include <random>
#include <cmath>
#include <xmmintrin.h>
#include "LibMath.h"
#ifdef _M_X64
#define NO_SIMD_ASM
#endif
#ifndef _MSC_VER
#define NO_SIMD_ASM
#endif

namespace VectorMath
{
	using namespace CoreLib::Basic;
	const float PI = 3.1415926535f;
	const float Epsilon = 1e-4f;
	const int DefaultFloatUlps = 1024;
	inline float Clamp(float val, float vmin, float vmax)
	{
		return val>vmax?vmax:val<vmin?vmin:val;
	}
	inline bool FloatEquals(float A, float B, int maxUlps = DefaultFloatUlps)
	{
		int aInt = *(int*)&A;
		// Make aInt lexicographically ordered as a twos-complement int
		if (aInt < 0)
			aInt = 0x80000000 - aInt;
		// Make bInt lexicographically ordered as a twos-complement int
		int bInt = *(int*)&B;
		if (bInt < 0)
			bInt = 0x80000000 - bInt;
		int intDiff = abs(aInt - bInt);
		if (intDiff <= maxUlps)
			return true;
		return false;
	}
	inline bool FloatLarger(float A, float B, int maxUlps = DefaultFloatUlps)
	{
		return A>B && !FloatEquals(A,B,maxUlps);
	}
	inline bool FloatSmaller(float A, float B, int maxUlps = DefaultFloatUlps)
	{
		return A<B && !FloatEquals(A,B,maxUlps);
	}
	inline bool FloatSmallerOrEquals(float A, float B, int maxUlps = DefaultFloatUlps)
	{
		return A<B || FloatEquals(A, B, maxUlps);
	}
	inline bool FloatLargerOrEquals(float A, float B, int maxUlps = DefaultFloatUlps)
	{
		return A>B || FloatEquals(A, B, maxUlps);
	}

	template<typename T>
	inline T Max(T v1, T v2)
	{
		if (v1>v2) return v1; else return v2;
	}
	template<typename T>
	inline T Min(T v1, T v2)
	{
		if (v1<v2) return v1; else return v2;
	}

	class Vec4;
	class Vec2
	{
	public:
		float x, y;
		Vec2()
		{}
		Vec2(float vx, float vy)
		{
			x = vx; y = vy;
		}
		Vec2(const Vec2 & v)
		{
			x = v.x;
			y = v.y;
		}
		inline void SetZero()
		{
			x = y = 0.0f;
		}
		inline float & operator [] (int i)
		{
			return ((float*)this)[i];
		}
		Vec2 operator *(float s)
		{
			Vec2 rs;
			rs.x = x * s;
			rs.y = y * s;
			return rs;
		}
	
	};

	struct Vec3_Struct
	{
		float x,y,z;
	};

	class Vec3 : public Vec3_Struct
	{
	public:
		inline Vec3()
		{}
		inline Vec3(float f)
		{
			x = y = z = f;
		}
		inline Vec3(float vx, float vy, float vz)
		{
			x = vx;	y = vy;	z = vz;
		}
		inline Vec3(const Vec3 & v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
		}
		static inline Vec3 FromHomogeneous(const Vec4 & v);
		inline void SetZero()
		{
			x = y = z = 0.0f;
		}
		inline float& operator [] (int i) const
		{
			return ((float*)this)[i];
		}
		inline Vec3 operator + (const Vec3 &vin)
		{
			Vec3 rs;
			rs.x = x + vin.x;
			rs.y = y + vin.y;
			rs.z = z + vin.z;
			return rs;
		}
		inline Vec3 operator - (const Vec3 &vin)
		{
			Vec3 rs;
			rs.x = x - vin.x;
			rs.y = y - vin.y;
			rs.z = z - vin.z;
			return rs;
		}
		inline Vec3 operator - ()
		{
			Vec3 rs;
			rs.x = -x;
			rs.y = -y;
			rs.z = -z;
			return rs;
		}
		inline Vec3 operator * (float scale)
		{
			Vec3 rs;
			rs.x = x * scale;
			rs.y = y * scale;
			rs.z = z * scale;
			return rs;
		}
		inline Vec3 & operator += (const Vec3 & vin)
		{
			x += vin.x; y += vin.y; z += vin.z;
			return *this;
		}
		inline Vec3 & operator -= (const Vec3 & vin)
		{
			x -= vin.x; y -= vin.y; z -= vin.z; 
			return *this;
		}
		inline Vec3 & operator *= (const Vec3 & vin)
		{
			x *= vin.x; y *= vin.y; z *= vin.z;
			return *this;
		}
		inline Vec3 & operator *= (float s)
		{
			x *= s; y *= s; z *= s;
			return *this;
		}
		inline Vec3 & operator /= (const Vec3 & vin)
		{
			x /= vin.x; y /= vin.y; z /= vin.z;
			return *this;
		}
		inline Vec3 & operator /= (float s)
		{
			float inv = 1.0f/s;
			return (*this)*=inv;
		}
		inline bool operator == (const Vec3 & vin)
		{
			return x == vin.x && y == vin.y && z == vin.z;
		}
		inline bool operator != (const Vec3 & vin)
		{
			return x != vin.x || y != vin.y || z != vin.z;
		}
		inline int GetHashCode()
		{
			return FloatAsInt(x) ^ FloatAsInt(y) ^ FloatAsInt(z);
		}
		inline static float Dot(const Vec3 & v1, const Vec3 & v2)
		{
			return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
		}
		inline static void Cross(Vec3 & rs_d, const Vec3 & v1, const Vec3 & v2)
		{
			rs_d.x = v1.y*v2.z - v1.z * v2.y;
			rs_d.y = v1.z*v2.x - v1.x * v2.z;
			rs_d.z = v1.x*v2.y - v1.y * v2.x;
		}
		inline static void Scale(Vec3 & rs, const Vec3 & v1, float s)
		{
			rs.x = v1.x*s;	rs.y = v1.y*s;	rs.z = v1.z*s;
		}
		inline static void Add(Vec3 & rs, const Vec3 & v1, const Vec3 & v2)
		{
			rs.x = v1.x + v2.x;
			rs.y = v1.y + v2.y;
			rs.z = v1.z + v2.z;
		}
		inline static void Subtract(Vec3 & rs, const Vec3 & v1, const Vec3 & v2)
		{
			rs.x = v1.x - v2.x;
			rs.y = v1.y - v2.y;
			rs.z = v1.z - v2.z;
		}
		inline static void Multiply(Vec3 & rs, const Vec3 & v1, const Vec3 & v2)
		{
			rs.x = v1.x * v2.x;
			rs.y = v1.y * v2.y;
			rs.z = v1.z * v2.z;
		}
		inline float LengthFPU() const
		{
			return sqrt(x*x + y*y + z*z);
		}
		inline float Length2() const
		{
			return x*x+y*y+z*z;
		}
		static inline void NormalizeFPU(Vec3 & rs, const Vec3 & vin)
		{
			float invLen = 1.0f/vin.LengthFPU();
			Scale(rs, vin, invLen);
		}
		inline float Length() const;
		static inline void Normalize(Vec3 & rs, const Vec3 & vin);
		inline Vec3 Normalize()
		{
			Vec3 rs;
			Normalize(rs, *this);
			return rs;
		}
	};

	struct Vec4_Struct
	{
		float x,y,z,w;
	};

	class Vec4 : public Vec4_Struct
	{
	public:
		inline Vec4()
		{}
		inline Vec4(const Vec4_Struct & v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
			w = v.w;
		}
		inline Vec4(float f)
		{
			x = y = z = w = f;
		}
		inline Vec4(float vx, float vy, float vz, float vw)
		{
			x = vx;	y = vy;	z = vz;	w = vw;
		}
		inline Vec4(const Vec3 & v)
		{
			x = v.x; y = v.y; z = v.z; w = 0.0f;
		}
		inline Vec4(const Vec3 & v, float vw)
		{
			x = v.x; y = v.y; z = v.z; w = vw;
		}
		inline Vec4(const Vec4 & v)
		{
			x = v.x; y = v.y; z = v.z; w = v.w;
		}
		inline void SetZero()
		{
			x = y = z = w = 0.0f;
		}
		inline void xyz(Vec3 & v) const
		{
			v.x = x;
			v.y = y;
			v.z = z;
		}
		inline Vec3 xyz() const
		{
			Vec3 rs;
			rs.x = x;
			rs.y = y;
			rs.z = z;
			return rs;
		}
		inline float& operator [] (int i)
		{
			return ((float*)this)[i];
		}
		inline Vec4 operator + (const Vec4 &vin)
		{
			Vec4 rs;
			rs.x = x + vin.x;
			rs.y = y + vin.y;
			rs.z = z + vin.z;
			rs.w = w + vin.w;
			return rs;
		}
		inline Vec4 operator - (const Vec4 &vin)
		{
			Vec4 rs;
			rs.x = x - vin.x;
			rs.y = y - vin.y;
			rs.z = z - vin.z;
			rs.w = w - vin.w;
			return rs;
		}
		inline Vec4 operator - ()
		{
			Vec4 rs;
			rs.x = -x;
			rs.y = -y;
			rs.z = -z;
			rs.w = -w;
			return rs;
		}
		inline Vec4 operator * (float scale) const
		{
			Vec4 rs;
			rs.x = x * scale;
			rs.y = y * scale;
			rs.z = z * scale;
			rs.w = w * scale;
			return rs;
		}
		inline Vec4 & operator += (const Vec4 & vin)
		{
			x += vin.x; y += vin.y; z += vin.z; w += vin.w;
			return *this;
		}
		inline Vec4 & operator -= (const Vec4 & vin)
		{
			x -= vin.x; y -= vin.y; z -= vin.z; w -= vin.w;
			return *this;
		}
		inline Vec4 & operator *= (const Vec4 & vin)
		{
			x *= vin.x; y *= vin.y; z *= vin.z; w *= vin.w;
			return *this;
		}
		inline Vec4 & operator *= (float s)
		{
			x *= s; y *= s; z *= s; w *= s;
			return *this;
		}
		inline Vec4 & operator /= (const Vec4 & vin)
		{
			x /= vin.x; y /= vin.y; z /= vin.z; w /= vin.w;
			return *this;
		}
		inline Vec4 & operator /= (float s)
		{
			float inv = 1.0f/s;
			return (*this)*=inv;
		}
		inline bool operator == (const Vec4 & vin)
		{
			return fabs(vin.x - x) < fabs(x)*Epsilon && 
				fabs(vin.y - y) < fabs(y)*Epsilon &&
				fabs(vin.z - z) < fabs(z)*Epsilon;
		}
		inline bool operator != (const Vec4 & vin)
		{
			return fabs(vin.x - x) > fabs(x)*Epsilon ||
				fabs(vin.y - y) > fabs(y)*Epsilon ||
				fabs(vin.z - z) > fabs(z)*Epsilon;
		}
		inline int GetHashCode()
		{
			return FloatAsInt(x) ^ FloatAsInt(y) ^ FloatAsInt(z) ^ FloatAsInt(w);
		}
		static inline void Add(Vec4 & rs, const Vec4 & v1, const Vec4 & v2);
		static inline void Subtract(Vec4 & rs, const Vec4 & v1, const Vec4 & v2);
		static inline void Multiply(Vec4 & rs, const Vec4 & v1, const Vec4 & v2);
		static inline void MultiplyScale(Vec4 & rs, const Vec4 & v1, const Vec4 & v2);
		static inline void Scale(Vec4 & rs, const Vec4 & v1, float s);
		static inline float Dot(const Vec4 & v1, const Vec4 & v2);
		static inline void Cross(Vec4 & rs_d, const Vec4 & v1, const Vec4 & v2);
		inline float LengthFPU() const;
		inline float Length() const;
		static inline void NormalizeFPU(Vec4& vout, const Vec4& vin);
		static inline void Normalize(Vec4 &vout, const Vec4 &vin);
		inline Vec4 Normalize()
		{
			Vec4 rs;
			Normalize(rs, *this);
			return rs;
		}
	};

	class Vec4_M128
	{
	public:
		__m128 vec;
		Vec4_M128()
		{}
		Vec4_M128(__m128 v)
		{
			vec = v;
		}
		Vec4_M128(float a, float b, float c, float d)
		{
			vec = _mm_set_ps(a, b, c, d);
		}
		Vec4_M128(const Vec4 & v)
		{
			vec = _mm_load_ps((const float*)&v);
		}
		inline void Zero()
		{
			vec = _mm_setzero_ps();
		}
		inline void ToVec4(Vec4 & v) const
		{
			_mm_store_ps((float*)&v, vec);
		}
	};

	class Matrix4
	{
	public:
		union
		{
			float values[16];
			float m[4][4];
			struct
			{
				float _11,_12,_13,_14,
				  _21,_22,_23,_24,
				  _31,_32,_33,_34,
				  _41,_42,_43,_44;
			} mi;
			struct
			{
				float _11,_12,_13,_14,
				  _21,_22,_23,_24,
				  _31,_32,_33,_34,
				  _41,_42,_43,_44;
			} mr;
		};
		Matrix4()
		{}
		Matrix4(float v)
		{
			for (int i = 0; i<16; i++)
				values[i] = v;
		}
		Matrix4(const Vec4 & c1, const Vec4 & c2, const Vec4 & c3, const Vec4 &c4)
		{
			memcpy(m[0], &c1, sizeof(Vec4));
			memcpy(m[1], &c2, sizeof(Vec4));
			memcpy(m[2], &c3, sizeof(Vec4));
			memcpy(m[3], &c4, sizeof(Vec4));
		}
		inline Matrix4 & operator *= (const Matrix4 & m)
		{
			Multiply(*this, *this, m);
			return *this;
		}
		inline Matrix4 & LeftMultiply(const Matrix4 & m)
		{
			Multiply(*this, m, *this);
			return *this;
		}
		inline void Transpose()
		{
			float tmp;
			for (int i = 1; i<4; i++)
				for (int j = 0; j < i; j++)
				{
					tmp = m[i][j];
					m[i][j] = m[j][i];
					m[j][i] = tmp;
				}
		}
		static inline void CreateIdentityMatrix(Matrix4 & mOut);
		static inline void CreateRandomMatrix(Matrix4 & mOut);
		static inline void CreatePerspectiveMatrixFromViewAngle(Matrix4 &mOut, float fovY, float aspect, float zNear, float zFar);
		static inline void CreatePerspectiveMatrix(Matrix4 &mOut, float left, float right, float bottom, float top, float zNear, float zFar);
		static void LookAt(Matrix4 & rs, const Vec3 & pos, const Vec3 & center, const Vec3 & up);
		static inline void RotationX(Matrix4 & rs, float angle);
		static inline void RotationY(Matrix4 & rs, float angle);
		static inline void RotationZ(Matrix4 & rs, float angle);
		static void Rotation(Matrix4 & rs, const Vec3 & axis, float angle);
		static void Rotation(Matrix4 & rs, float yaw, float pitch, float roll);
		static inline void Scale(Matrix4 & rs, float sx, float sy, float sz);
		static inline void Translation(Matrix4 & rs, float tx, float ty, float tz);
		inline void Transform(Vec3 & rs_d, const Vec3& vIn) const;
		inline void Transform(Vec4 & rs_d, const Vec4& vIn) const;
		inline void TransformNormal(Vec3 & rs, const Vec3& vIn) const;
		inline void TransposeTransformNormal(Vec3 & rs, const Vec3 & vIn) const;
		inline void TransposeTransform(Vec3 & rs, const Vec3 & vIn) const;
		inline void TransposeTransform(Vec4 & rs_d, const Vec4& vIn) const;
		inline void TransformHomogeneous(Vec3 & rs, const Vec3 & vIn) const;
		inline void TransformHomogeneous2D(Vec2 & rs, const Vec3 & vIn) const;
		static inline void MultiplyFPU(Matrix4 &mOut, const Matrix4& M1, const Matrix4& M2);
		static inline void Multiply(Matrix4 &mOut, const Matrix4& M1, const Matrix4& M2);
		float Inverse3D(Matrix4 & mOut_d) const;
		float InverseFPU(Matrix4 &mOut_d) const;
		inline float Inverse(Matrix4 &mOut_d) const;
	};

	//__declspec(align(16))
	class Matrix4_M128
	{
	private:
		static const __m128 VecOne;
	public:
		__m128 C1,C2,C3,C4;
		Matrix4_M128()
		{}
		Matrix4_M128(const Matrix4 & m)
		{
			C1 = _mm_loadu_ps(m.values);
			C2 = _mm_loadu_ps(m.values+4);
			C3 = _mm_loadu_ps(m.values+8);
			C4 = _mm_loadu_ps(m.values+12);
		}
		inline void ToMatrix4(Matrix4 & mOut) const;
		inline void Transform(Vec4_M128 & rs, const Vec4& vIn) const;
		inline void Transform(Vec4 & rs, const Vec4& vIn) const;
		inline void Transform(Vec4_M128 & rs, const Vec3& vIn) const;
		inline void Transform(Vec3 & rs, const Vec3& vIn) const;
		inline void Transform(Vec4_M128 & rs, const Vec4_M128& vIn) const;
		inline void TransformNormal(Vec4_M128 & rs, const Vec4& vIn) const;
		inline void TransformNormal(Vec4 & rs, const Vec4& vIn) const;
		inline void TransformNormal(Vec4_M128 & rs, const Vec3& vIn) const;
		inline void TransformNormal(Vec3 & rs, const Vec3& vIn) const;
		inline void Multiply(Matrix4_M128 & rs, const Matrix4 & mB) const;
		inline void Multiply(Matrix4_M128 & rs, const Matrix4_M128 & mB) const;
		float Inverse(Matrix4_M128 &mOut) const;
	};

	//***********************************************************************
	/**************************** Implementation ****************************/
	//***********************************************************************
	//inline int FloatAsInt(float val)
	//{
	//	union InterCast
	//	{
	//		float fvalue;
	//		int ivalue;
	//	} cast;
	//	cast.fvalue = val;
	//	return cast.ivalue;
	//}
	//inline float IntAsFloat(int val)
	//{
	//	union InterCast
	//	{
	//		float fvalue;
	//		int ivalue;
	//	} cast;
	//	cast.ivalue = val;
	//	return cast.fvalue;
	//}
	inline Vec3 Vec3::FromHomogeneous(const Vec4 & v)
	{
		float invW = 1.0f/v.w;
		return v.xyz()*invW;
	}
	// Vec3
	inline float Vec3::Length() const
	{
		return Vec4(*this).Length();
	}
	inline void Vec3::Normalize(Vec3 & rs, const Vec3 & vin)
	{
		Vec3::NormalizeFPU(rs, vin);
	}
	// Vec4
	inline void Vec4::Add(Vec4 & rs, const Vec4 & v1, const Vec4 & v2)
	{
		rs.x = v1.x + v2.x;
		rs.y = v1.y + v2.y;
		rs.z = v1.z + v2.z;
		rs.w = v1.w + v2.w;
	}
	inline void Vec4::Subtract(Vec4 & rs, const Vec4 & v1, const Vec4 & v2)
	{
		rs.x = v1.x - v2.x;
		rs.y = v1.y - v2.y;
		rs.z = v1.z - v2.z;
		rs.w = v1.w - v2.w;
	}
	inline void Vec4::Multiply(Vec4 & rs, const Vec4 & v1, const Vec4 & v2)
	{
		rs.x = v1.x * v2.x;
		rs.y = v1.y * v2.y;
		rs.z = v1.z * v2.z;
		rs.w = v1.w * v2.w;
	}
	inline void Vec4::MultiplyScale(Vec4 & rs, const Vec4 & v1, const Vec4 & v2)
	{
		rs.x = v1.x * v2.x;
		rs.y = v1.y * v2.y;
		rs.z = v1.z * v2.z;
		rs.w = v1.w * v2.w;
	}
	inline void Vec4::Scale(Vec4 & rs, const Vec4 & v1, float s)
	{
		rs.x = v1.x * s;
		rs.y = v1.y * s;
		rs.z = v1.z * s;
		rs.w = v1.w * s;
	}
	inline float Vec4::Dot(const Vec4 & v1, const Vec4 & v2)
	{
		return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z + v1.w*v2.w;
	}
	inline void Vec4::Cross(Vec4 & rs_d, const Vec4 & v1, const Vec4 & v2)
	{
		rs_d.x = v1.y*v2.z - v1.z * v2.y;
		rs_d.y = v1.z*v2.x - v1.x * v2.z;
		rs_d.z = v1.x*v2.y - v1.y * v2.x;
		rs_d.w = 0.0f;
	}
	inline float Vec4::LengthFPU() const
	{
		return sqrt(Dot(*this, *this));
	}
	inline float Vec4::Length() const
	{
#ifdef NO_SIMD_ASM
		return LengthFPU();
#else
		float f;
		_asm
		{
			lea	ecx, f;
			mov	eax, this;

			movups	xmm0, [eax];
			mulps	xmm0, xmm0;
			movaps	xmm1, xmm0;
			shufps	xmm1, xmm1, 4Eh;
			addps	xmm0, xmm1;
			movaps	xmm1, xmm0;
			shufps	xmm1, xmm1, 11h;
			addss	xmm0, xmm1;

			sqrtss	xmm0, xmm0;
			movss	dword ptr [ecx], xmm0;
		}
		return f;
#endif
	}
	inline void Vec4::NormalizeFPU(Vec4& vout, const Vec4& vin)
	{
		float len = 1.0f/vin.Length();
		Scale(vout, vin, len);
	}
	inline void Vec4::Normalize(Vec4 &vout, const Vec4 &vin)
	{
#ifdef NO_SIMD_ASM
		NormalizeFPU(vout, vin);
#else
		vout = vin;
		void * v_ptr = &vout;
		const void * v_ptr_in = &vin;
		_asm
		{
			movups	xmm0, [v_ptr_in];
			movaps	xmm2, xmm0;
			mulps	xmm0, xmm0;
			movaps	xmm1, xmm0;
			shufps	xmm1, xmm1, 4Eh;
			addps	xmm0, xmm1;
			movaps	xmm1, xmm0;
			shufps	xmm1, xmm1, 11h;
			addps	xmm0, xmm1;

			rsqrtps	xmm0, xmm0;
			mulps	xmm2, xmm0;
			movups	[v_ptr], xmm2;
		}
#endif
	}

	// Matrix4
	inline void Matrix4::CreateIdentityMatrix(Matrix4 & mOut)
	{
		memset(&mOut, 0, sizeof(Matrix4));
		mOut.m[0][0] = mOut.m[1][1] = mOut.m[2][2] = mOut.m[3][3] = 1.0f;
	}

	inline void Matrix4::CreatePerspectiveMatrix(Matrix4 &mOut, float left, float right, float bottom, float top, float znear, float zfar)
	{
		memset(&mOut, 0, sizeof(Matrix4));
		mOut.m[0][0] = (znear*2.0f)/(right-left);
		mOut.m[1][1] = (2.0f*znear)/(top-bottom);
		mOut.m[2][0] = (right+left)/(right-left);
		mOut.m[2][1] = (top+bottom)/(top-bottom);
		mOut.m[2][2] = (zfar+znear)/(znear-zfar);
		mOut.m[2][3] = -1.0f;
		mOut.m[3][2] = 2.0f*zfar*znear/(znear-zfar);
	}

	inline void Matrix4::CreatePerspectiveMatrixFromViewAngle(Matrix4 &mOut, float fovY, float aspect, float zNear, float zFar)
	{
		float xmin, xmax, ymin, ymax;
		ymax = zNear * tan(fovY * CoreLib::Basic::Math::Pi / 360.0f);
		ymin = -ymax;
		xmin = ymin * aspect;
		xmax = ymax * aspect;
		Matrix4::CreatePerspectiveMatrix(mOut, xmin, xmax, ymin, ymax, zNear, zFar);
	}

	inline void Matrix4::CreateRandomMatrix(Matrix4 & mOut)
	{
		for (int i = 0; i<16; i++)
		{
			mOut.values[i] = rand()/(float)RAND_MAX;
		}
	}
	inline void Matrix4::RotationX(Matrix4 & rs, float angle)
	{
		float c = cosf(angle);
		float s = sinf(angle);

		Matrix4::CreateIdentityMatrix(rs);
		rs.m[1][1] = c;
		rs.m[2][1] = s;
		rs.m[1][2] = -s;
		rs.m[2][2] = c;
	}
	inline void Matrix4::RotationY(Matrix4 & rs, float angle)
	{
		float c = cosf(angle);
		float s = sinf(angle);

		Matrix4::CreateIdentityMatrix(rs);
		rs.m[0][0] = c;
		rs.m[2][0] = s;
		rs.m[0][2] = -s;
		rs.m[2][2] = c;
	}
	inline void Matrix4::RotationZ(Matrix4 & rs, float angle)
	{
		float c = cosf(angle);
		float s = sinf(angle);

		Matrix4::CreateIdentityMatrix(rs);
		rs.m[0][0] = c;
		rs.m[1][0] = s;
		rs.m[0][1] = -s;
		rs.m[1][1] = c;
	}

	inline void Matrix4::Scale(Matrix4 & rs, float sx, float sy, float sz)
	{
		Matrix4::CreateIdentityMatrix(rs);
		rs.m[0][0] = sx;
		rs.m[1][1] = sy;
		rs.m[2][2] = sz;
	}
	inline void Matrix4::Translation(Matrix4 & rs, float tx, float ty, float tz)
	{
		Matrix4::CreateIdentityMatrix(rs);
		rs.values[12] = tx;
		rs.values[13] = ty;
		rs.values[14] = tz;
	}
	inline void Matrix4::TransposeTransformNormal(Vec3 & rs, const Vec3 & vIn) const
	{
		rs.x = m[0][0]*vIn.x + m[0][1]*vIn.y + m[0][2]*vIn.z;
		rs.y = m[1][0]*vIn.x + m[1][1]*vIn.y + m[1][2]*vIn.z;
		rs.z = m[2][0]*vIn.x + m[2][1]*vIn.y + m[2][2]*vIn.z;
	}
	inline void Matrix4::TransposeTransform(Vec3 & rs, const Vec3 & vIn) const
	{
		rs.x = m[0][0]*vIn.x + m[0][1]*vIn.y + m[0][2]*vIn.z + m[0][3];
		rs.y = m[1][0]*vIn.x + m[1][1]*vIn.y + m[1][2]*vIn.z + m[1][3];
		rs.z = m[2][0]*vIn.x + m[2][1]*vIn.y + m[2][2]*vIn.z + m[2][3];
	}
	inline void Matrix4::TransposeTransform(Vec4 & rs, const Vec4 & vIn) const
	{
		rs.x = m[0][0]*vIn.x + m[0][1]*vIn.y + m[0][2]*vIn.z + m[0][3]*vIn.w;
		rs.y = m[1][0]*vIn.x + m[1][1]*vIn.y + m[1][2]*vIn.z + m[1][3]*vIn.w;
		rs.z = m[2][0]*vIn.x + m[2][1]*vIn.y + m[2][2]*vIn.z + m[2][3]*vIn.w;
		rs.w = m[3][0]*vIn.x + m[3][1]*vIn.y + m[3][2]*vIn.z + m[3][3]*vIn.w;
	}
	inline void Matrix4::Transform(Vec3 & rs, const Vec3& vIn) const
	{
		rs.x = m[0][0]*vIn.x + m[1][0]*vIn.y + m[2][0]*vIn.z + m[3][0];
		rs.y = m[0][1]*vIn.x + m[1][1]*vIn.y + m[2][1]*vIn.z + m[3][1];
		rs.z = m[0][2]*vIn.x + m[1][2]*vIn.y + m[2][2]*vIn.z + m[3][2];
	}
	inline void Matrix4::TransformHomogeneous(Vec3 & rs, const Vec3 & vIn) const
	{
		rs.x = m[0][0]*vIn.x + m[1][0]*vIn.y + m[2][0]*vIn.z + m[3][0];
		rs.y = m[0][1]*vIn.x + m[1][1]*vIn.y + m[2][1]*vIn.z + m[3][1];
		rs.z = m[0][2]*vIn.x + m[1][2]*vIn.y + m[2][2]*vIn.z + m[3][2];
		float w = 1.0f/(m[0][3]*vIn.x + m[1][3]*vIn.y + m[2][3]*vIn.z + m[3][3]);
		rs.x *= w;
		rs.y *= w;
		rs.z *= w;
	}
	inline void Matrix4::TransformHomogeneous2D(Vec2 & rs, const Vec3 & vIn) const
	{
		rs.x = m[0][0]*vIn.x + m[1][0]*vIn.y + m[2][0]*vIn.z + m[3][0];
		rs.y = m[0][1]*vIn.x + m[1][1]*vIn.y + m[2][1]*vIn.z + m[3][1];
		float w = 1.0f/(m[0][3]*vIn.x + m[1][3]*vIn.y + m[2][3]*vIn.z + m[3][3]);
		rs.x *= w;
		rs.y *= w;
	}
	inline void Matrix4::TransformNormal(Vec3 & rs, const Vec3& vIn) const
	{
		rs.x = m[0][0]*vIn.x + m[1][0]*vIn.y + m[2][0]*vIn.z;
		rs.y = m[0][1]*vIn.x + m[1][1]*vIn.y + m[2][1]*vIn.z;
		rs.z = m[0][2]*vIn.x + m[1][2]*vIn.y + m[2][2]*vIn.z;
	}
	inline void Matrix4::Transform(Vec4 & rs, const Vec4& vIn) const
	{
		rs.x = m[0][0]*vIn.x + m[1][0]*vIn.y + m[2][0]*vIn.z + m[3][0]*vIn.w;
		rs.y = m[0][1]*vIn.x + m[1][1]*vIn.y + m[2][1]*vIn.z + m[3][1]*vIn.w;
		rs.z = m[0][2]*vIn.x + m[1][2]*vIn.y + m[2][2]*vIn.z + m[3][2]*vIn.w;
		rs.w = m[0][3]*vIn.x + m[1][3]*vIn.y + m[2][3]*vIn.z + m[3][3]*vIn.w;
	}
	inline void Matrix4::MultiplyFPU(Matrix4 &mOut, const Matrix4& M1, const Matrix4& M2)
	{
		Matrix4 TempMat;
		for (int i=0;i<4;i++) //col
		{
			for (int j=0;j<4;j++) // row
			{
				TempMat.m[i][j] = M1.m[0][j]*M2.m[i][0] + M1.m[1][j]*M2.m[i][1] + M1.m[2][j]*M2.m[i][2] + M1.m[3][j]*M2.m[i][3];
			}
		}
		memcpy(&mOut,&TempMat,sizeof(Matrix4));
	}

	inline void Matrix4::Multiply(Matrix4 &mOut, const Matrix4 &M1, const Matrix4 &M2)
	{
		Matrix4 rs;
		Matrix4_M128 TempMat;
		Matrix4_M128 mA(M1);
		mA.Multiply(TempMat, M2);
		TempMat.ToMatrix4(rs);
		mOut = rs;
	}
	inline float Matrix4::Inverse(Matrix4 &mOut_d) const
	{
		Matrix4 mat;
		Matrix4_M128 m_m(*this);
		Matrix4_M128 mr;
		float rs = m_m.Inverse(mr);
		mr.ToMatrix4(mat);
		mOut_d = mat;
		return rs;
	}

	// Matrix4_M128

	inline void Matrix4_M128::ToMatrix4(Matrix4 & mOut) const
	{
		_mm_storeu_ps(mOut.values, C1);
		_mm_storeu_ps(mOut.values+4, C2);
		_mm_storeu_ps(mOut.values+8, C3);
		_mm_storeu_ps(mOut.values+12, C4);
	}
	inline void Matrix4_M128::Transform(Vec4_M128 & rs, const Vec4& vIn) const
	{
		__m128 r;
		r = _mm_mul_ps(C1, _mm_set_ps1(vIn.x));
		r = _mm_add_ps(r, _mm_mul_ps(C2, _mm_set_ps1(vIn.y)));
		r = _mm_add_ps(r, _mm_mul_ps(C3, _mm_set_ps1(vIn.z)));
		r = _mm_add_ps(r, _mm_mul_ps(C4, _mm_set_ps1(vIn.w)));
		rs.vec = r;
	}
	inline void Matrix4_M128::Transform(Vec4 & rs, const Vec4& vIn) const
	{
		Vec4_M128 r;
		Transform(r, vIn);
		_mm_store_ps((float*)&rs, r.vec);
	}
	inline void Matrix4_M128::Transform(Vec4_M128 & rs, const Vec3& vIn) const
	{
		__m128 r;
		r = _mm_mul_ps(C1, _mm_set_ps1(vIn.x));
		r = _mm_add_ps(r, _mm_mul_ps(C2, _mm_set_ps1(vIn.y)));
		r = _mm_add_ps(r, _mm_mul_ps(C3, _mm_set_ps1(vIn.z)));
		rs.vec = r;
	}
	inline void Matrix4_M128::Transform(Vec3 & rs, const Vec3& vIn) const
	{
		Vec4_M128 r;
		Transform(r, vIn);
		Vec4 tmp;
		_mm_store_ps((float*)&tmp, r.vec);
		rs.x = tmp.x;
		rs.y = tmp.y;
		rs.z = tmp.z;
	}
	inline void Matrix4_M128::Transform(Vec4_M128 & rs, const Vec4_M128& vIn) const
	{
		__m128 r;
		__m128 x,y,z,w;
		x = _mm_shuffle_ps(vIn.vec, vIn.vec, _MM_SHUFFLE(0, 0, 0, 0));
		r = _mm_mul_ps(C1, x);
		y = _mm_shuffle_ps(vIn.vec, vIn.vec, _MM_SHUFFLE(1, 1, 1, 1));
		r = _mm_add_ps(r, _mm_mul_ps(C2, y));
		z = _mm_shuffle_ps(vIn.vec, vIn.vec, _MM_SHUFFLE(2, 2, 2, 2));
		r = _mm_add_ps(r, _mm_mul_ps(C3, z));
		w = _mm_shuffle_ps(vIn.vec, vIn.vec, _MM_SHUFFLE(3, 3, 3, 3));
		r = _mm_add_ps(r, _mm_mul_ps(C4, w));
		rs.vec = r;
	}
	inline void Matrix4_M128::TransformNormal(Vec4_M128 & rs, const Vec4& vIn) const
	{
		__m128 r;
		r = _mm_mul_ps(C1, _mm_set_ps1(vIn.x));
		r = _mm_add_ps(r, _mm_mul_ps(C2, _mm_set_ps1(vIn.y)));
		r = _mm_add_ps(r, _mm_mul_ps(C3, _mm_set_ps1(vIn.z)));
		rs.vec = r;
	}
	inline void Matrix4_M128::TransformNormal(Vec4 & rs, const Vec4& vIn) const
	{
		Vec4_M128 r;
		TransformNormal(r, vIn);
		_mm_store_ps((float*)&rs, r.vec);
		rs.w = 0.0f;
	}
	inline void Matrix4_M128::TransformNormal(Vec4_M128 & rs, const Vec3& vIn) const
	{
		__m128 r;
		r = _mm_mul_ps(C1, _mm_set_ps1(vIn.x));
		r = _mm_add_ps(r, _mm_mul_ps(C2, _mm_set_ps1(vIn.y)));
		r = _mm_add_ps(r, _mm_mul_ps(C3, _mm_set_ps1(vIn.z)));
		rs.vec = r;
	}
	inline void Matrix4_M128::TransformNormal(Vec3 & rs, const Vec3& vIn) const
	{
		Vec4_M128 r;
		TransformNormal(r, vIn);
		Vec4 tmp;
		_mm_store_ps((float*)&tmp, r.vec);
		rs = tmp.xyz();
	}
	inline void Matrix4_M128::Multiply(Matrix4_M128 & rs, const Matrix4 & mB) const
	{
		register __m128 T0, T1, T2, T3, R0, R1, R2, R3;
		T0 = _mm_set_ps1(mB.values[0]);
		T1 = _mm_set_ps1(mB.values[1]);
		T2 = _mm_set_ps1(mB.values[2]);
		T3 = _mm_set_ps1(mB.values[3]);
		R0 = _mm_mul_ps(C1, T0);
		T0 = _mm_set_ps1(mB.values[4]);
		R1 = _mm_mul_ps(C2, T1);
		R1 = _mm_add_ps(R1, R0);
		R2 = _mm_mul_ps(C3, T2);
		T1 = _mm_set_ps1(mB.values[5]);
		R3 = _mm_mul_ps(C4, T3);
		R2 = _mm_add_ps(R2, R1);
		T2 = _mm_set_ps1(mB.values[6]);
		rs.C1 = _mm_add_ps(R3, R2);
		R0 = _mm_mul_ps(C1, T0);
		T3 = _mm_set_ps1(mB.values[7]);
		R1 = _mm_mul_ps(C2, T1);
		T0 = _mm_set_ps1(mB.values[8]);
		R2 = _mm_mul_ps(C3, T2);
		R1 = _mm_add_ps(R1, R0);
		T1 = _mm_set_ps1(mB.values[9]);
		R3 = _mm_mul_ps(C4, T3);
		R2 = _mm_add_ps(R2, R1);
		rs.C2 = _mm_add_ps(R3, R2);
		T2 = _mm_set_ps1(mB.values[10]);
		R0 = _mm_mul_ps(C1, T0);
		T3 = _mm_set_ps1(mB.values[11]);
		R1 = _mm_mul_ps(C2, T1);
		T0 = _mm_set_ps1(mB.values[12]);
		R2 = _mm_mul_ps(C3, T2);
		R1 = _mm_add_ps(R1, R0);
		T2 = _mm_set_ps1(mB.values[14]);
		R3 = _mm_mul_ps(C4, T3);
		R2 = _mm_add_ps(R2, R1);
		T1 = _mm_set_ps1(mB.values[13]);
		rs.C3 = _mm_add_ps(R3, R2);
		R0 = _mm_mul_ps(C1, T0);
		R1 = _mm_mul_ps(C2, T1);
		T3 = _mm_set_ps1(mB.values[15]);
		R2 = _mm_mul_ps(C3, T2);
		R1 = _mm_add_ps(R1, R0);
		R3 = _mm_mul_ps(C4, T3);
		R2 = _mm_add_ps(R2, R1);
		rs.C4 = _mm_add_ps(R3, R2);
	}
	inline void Matrix4_M128::Multiply(Matrix4_M128 & rs, const Matrix4_M128 & mB) const
	{
		register __m128 T0, T1, T2, T3, R0, R1, R2, R3;
		T0 = _mm_shuffle_ps(mB.C1, mB.C1, _MM_SHUFFLE(0,0,0,0));
		T1 = _mm_shuffle_ps(mB.C1, mB.C1, _MM_SHUFFLE(1,1,1,1));
		T2 = _mm_shuffle_ps(mB.C1, mB.C1, _MM_SHUFFLE(2,2,2,2));
		T3 = _mm_shuffle_ps(mB.C1, mB.C1, _MM_SHUFFLE(3,3,3,3));
		R0 = _mm_mul_ps(C1, T0);
		R1 = _mm_mul_ps(C2, T1);
		R2 = _mm_mul_ps(C3, T2);
		R3 = _mm_mul_ps(C4, T3);
		R1 = _mm_add_ps(R1, R0);
		R2 = _mm_add_ps(R2, R1);
		rs.C1 = _mm_add_ps(R3, R2);

		T0 = _mm_shuffle_ps(mB.C2, mB.C2, _MM_SHUFFLE(0,0,0,0));
		T1 = _mm_shuffle_ps(mB.C2, mB.C2, _MM_SHUFFLE(1,1,1,1));
		T2 = _mm_shuffle_ps(mB.C2, mB.C2, _MM_SHUFFLE(2,2,2,2));
		T3 = _mm_shuffle_ps(mB.C2, mB.C2, _MM_SHUFFLE(3,3,3,3));
		R0 = _mm_mul_ps(C1, T0);
		R1 = _mm_mul_ps(C2, T1);
		R2 = _mm_mul_ps(C3, T2);
		R3 = _mm_mul_ps(C4, T3);
		R1 = _mm_add_ps(R1, R0);
		R2 = _mm_add_ps(R2, R1);
		rs.C2 = _mm_add_ps(R3, R2);

		T0 = _mm_shuffle_ps(mB.C3, mB.C3, _MM_SHUFFLE(0,0,0,0));
		T1 = _mm_shuffle_ps(mB.C3, mB.C3, _MM_SHUFFLE(1,1,1,1));
		T2 = _mm_shuffle_ps(mB.C3, mB.C3, _MM_SHUFFLE(2,2,2,2));
		T3 = _mm_shuffle_ps(mB.C3, mB.C3, _MM_SHUFFLE(3,3,3,3));
		R0 = _mm_mul_ps(C1, T0);
		R1 = _mm_mul_ps(C2, T1);
		R2 = _mm_mul_ps(C3, T2);
		R3 = _mm_mul_ps(C4, T3);
		R1 = _mm_add_ps(R1, R0);
		R2 = _mm_add_ps(R2, R1);
		rs.C3 = _mm_add_ps(R3, R2);

		T0 = _mm_shuffle_ps(mB.C4, mB.C4, _MM_SHUFFLE(0,0,0,0));
		T1 = _mm_shuffle_ps(mB.C4, mB.C4, _MM_SHUFFLE(1,1,1,1));
		T2 = _mm_shuffle_ps(mB.C4, mB.C4, _MM_SHUFFLE(2,2,2,2));
		T3 = _mm_shuffle_ps(mB.C4, mB.C4, _MM_SHUFFLE(3,3,3,3));
		R0 = _mm_mul_ps(C1, T0);
		R1 = _mm_mul_ps(C2, T1);
		R2 = _mm_mul_ps(C3, T2);
		R3 = _mm_mul_ps(C4, T3);
		R1 = _mm_add_ps(R1, R0);
		R2 = _mm_add_ps(R2, R1);
		rs.C4 = _mm_add_ps(R3, R2);
	}

	inline void CartesianToSphere(const Vec3 & dir, float & u, float & v)
	{
		const float inv2Pi = 0.5f/PI;
		v = acos(dir.y);
		u = atan2(dir.z, dir.x);
		if (u<0.0f)
			u += PI * 2.0f;
		u *= inv2Pi;
		v *= 1.0f/PI;
	}

	inline void SphereToCartesian(Vec3 & dir, float u, float v)
	{
		dir.y = cos(v);
		float s = sin(v);
		dir.x = cos(u) * s;
		dir.z = sin(u) * s;
	}

	inline void GetOrthoVec(Vec3 & vout, const Vec3 & vin)
	{
		Vec3 absV(abs(vin.x), abs(vin.y), abs(vin.z));
		if (absV.x <= absV.y && absV.x <= absV.z)
			Vec3::Cross(vout, vin, Vec3(1.0f, 0.0f, 0.0f));
		else if (absV.y <= absV.x && absV.y <= absV.z)
			Vec3::Cross(vout, vin, Vec3(0.0f, 1.0f, 0.0f));
		else
			Vec3::Cross(vout, vin, Vec3(0.0f, 0.0f, 1.0f));
	}
	inline __m128 _mm_neg(const __m128 & v)
	{
		static const __m128 SIGNMASK = 
               _mm_castsi128_ps(_mm_set1_epi32(0x80000000));
		return _mm_xor_ps(v, SIGNMASK);
	}
// GCC does not allow overloading sse vectors
#define M128_OPERATOR_OVERLOADS
#ifndef M128_OPERATOR_OVERLOADS
#define M128_OPERATOR_OVERLOADS
	inline __m128 & operator += (__m128 & v0, const __m128 &v1)
	{
		v0 = _mm_add_ps(v0, v1);
		return v0;
	}
	inline __m128 & operator -= (__m128 & v0, const __m128 &v1)
	{
		v0 = _mm_sub_ps(v0, v1);
		return v0;
	}
	inline __m128 & operator *= (__m128 & v0, const __m128 &v1)
	{
		v0 = _mm_mul_ps(v0, v1);
		return v0;
	}
	inline __m128 & operator /= (__m128 & v0, const __m128 &v1)
	{
		v0 = _mm_div_ps(v0, v1);
		return v0;
	}
	inline __m128 operator + (const __m128 & v0, const __m128 & v1)
	{
		return _mm_add_ps(v0, v1);
	}
	inline __m128 operator - (const __m128 & v0, const __m128 & v1)
	{
		return _mm_sub_ps(v0, v1);
	}
	inline __m128 operator * (const __m128 & v0, const __m128 & v1)
	{
		return _mm_mul_ps(v0, v1);
	}
	inline __m128 operator / (const __m128 & v0, const __m128 & v1)
	{
		return _mm_div_ps(v0, v1);
	}
	inline __m128 operator - (const __m128 & v0)
	{
		static const __m128 SIGNMASK = 
               _mm_castsi128_ps(_mm_set1_epi32(0x80000000));
		return _mm_xor_ps(v0, SIGNMASK);
	}

	inline __m128i & operator += (__m128i & v0, const __m128i &v1)
	{
		v0 = _mm_add_epi32(v0, v1);
		return v0;
	}
	inline __m128i & operator -= (__m128i & v0, const __m128i &v1)
	{
		v0 = _mm_sub_epi32(v0, v1);
		return v0;
	}
	inline __m128i & operator *= (__m128i & v0, const __m128i &v1)
	{
		v0 = _mm_mullo_epi32(v0, v1);
		return v0;
	}
	inline __m128i operator + (const __m128i & v0, const __m128i & v1)
	{
		return _mm_add_epi32(v0, v1);
	}
	inline __m128i operator - (const __m128i & v0, const __m128i & v1)
	{
		return _mm_sub_epi32(v0, v1);
	}
	inline __m128i operator * (const __m128i & v0, const __m128i & v1)
	{
		return _mm_mullo_epi32(v0, v1);
	}
	inline __m128i operator - (const __m128i & v0)
	{
		return _mm_xor_si128(v0, _mm_set1_epi32(0xFFFFFFFF));
	}

	_declspec(align(16))
	class SSEVec3
	{
	public:
		__m128 x,y,z;
		SSEVec3()
		{};
		SSEVec3(__m128 x, __m128 y, __m128 z)
			:x(x), y(y), z(z)
		{
		}
		SSEVec3(const Vec3 &v)
		{
			this->x = _mm_set_ps1(v.x);
			this->y = _mm_set_ps1(v.y);
			this->z = _mm_set_ps1(v.z);
		}
		SSEVec3(float x, float y, float z)
		{
			this->x = _mm_set_ps1(x);
			this->y = _mm_set_ps1(y);
			this->z = _mm_set_ps1(z);
		}
		inline __m128 Length()
		{
			return _mm_sqrt_ps(x*x + y*y + z*z);
		}
		inline void Normalize(__m128 one)
		{
			auto s = one / Length();
			x *= s;
			y *= s;
			z *= s;
		}
		inline SSEVec3 operator + (const SSEVec3 &vin)
		{
			SSEVec3 rs;
			rs.x = x + vin.x;
			rs.y = y + vin.y;
			rs.z = z + vin.z;
			return rs;
		}
		inline SSEVec3 operator - (const SSEVec3 &vin)
		{
			SSEVec3 rs;
			rs.x = x - vin.x;
			rs.y = y - vin.y;
			rs.z = z - vin.z;
			return rs;
		}
		inline SSEVec3 operator - ()
		{
			SSEVec3 rs;
			rs.x = -x;
			rs.y = -y;
			rs.z = -z;
			return rs;
		}
		inline SSEVec3 operator * (__m128 scale)
		{
			SSEVec3 rs;
			rs.x = x * scale;
			rs.y = y * scale;
			rs.z = z * scale;
			return rs;
		}
		inline SSEVec3 & operator += (const SSEVec3 & vin)
		{
			x += vin.x; y += vin.y; z += vin.z;
			return *this;
		}
		inline SSEVec3 & operator -= (const SSEVec3 & vin)
		{
			x -= vin.x; y -= vin.y; z -= vin.z; 
			return *this;
		}
		inline SSEVec3 & operator *= (const SSEVec3 & vin)
		{
			x *= vin.x; y *= vin.y; z *= vin.z;
			return *this;
		}
		inline SSEVec3 & operator *= (__m128 s)
		{
			x *= s; y *= s; z *= s;
			return *this;
		}
		inline SSEVec3 & operator /= (const SSEVec3 & vin)
		{
			x /= vin.x; y /= vin.y; z /= vin.z;
			return *this;
		}
		inline SSEVec3 & operator /= (float s)
		{
			float inv = 1.0f/s;
			return (*this)*=_mm_set_ps1(inv);
		}

		inline static __m128 Dot(const SSEVec3 & v1, const SSEVec3 & v2)
		{
			return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
		}
		inline static void Cross(SSEVec3 & rs_d, const SSEVec3 & v1, const SSEVec3 & v2)
		{
			rs_d.x = v1.y*v2.z - v1.z * v2.y;
			rs_d.y = v1.z*v2.x - v1.x * v2.z;
			rs_d.z = v1.x*v2.y - v1.y * v2.x;
		}
	};

	_declspec(align(16))
	class SSEMatrix4
	{
	public:
		__m128 values[16];
		SSEMatrix4()
		{}
		SSEMatrix4(const Matrix4 & mat)
		{
			for (int i = 0; i<16; i++)
				values[i] = _mm_set_ps1(mat.values[i]);
		}
		inline SSEVec3 Transform(SSEVec3 & v)
		{
			SSEVec3 rs;
			rs.x = values[0]*v.x + values[4]*v.y + values[8]*v.z + values[12];
			rs.y = values[1]*v.x + values[5]*v.y + values[9]*v.z + values[13];
			rs.z = values[2]*v.x + values[6]*v.y + values[10]*v.z + values[14];
			auto w = values[3]*v.x + values[7]*v.y + values[11]*v.z + values[15];
			w = _mm_set_ps1(1.0f) / w;
			rs.x *= w;
			rs.y *= w;
			rs.z *= w;
			return rs;
		}
		inline SSEVec3 TransformNonPerspective(SSEVec3 & v)
		{
			SSEVec3 rs;
			rs.x = values[0]*v.x + values[4]*v.y + values[8]*v.z + values[12];
			rs.y = values[1]*v.x + values[5]*v.y + values[9]*v.z + values[13];
			rs.z = values[2]*v.x + values[6]*v.y + values[10]*v.z + values[14];
			return rs;
		}
	};
#endif
}

#endif
