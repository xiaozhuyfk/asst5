#ifndef CORE_LIB_COMMON_H
#define CORE_LIB_COMMON_H

#include <cstdint>

#ifdef __GNUC__
#define CORE_LIB_ALIGN_16(x) x __attribute__((aligned(16)))
#else
#define CORE_LIB_ALIGN_16(x) __declspec(align(16)) x
#endif
namespace CoreLib
{
	typedef int64_t Int64;
	typedef unsigned short Word;

	namespace Basic
	{
		class Object
		{
		public:
			virtual ~Object()
			{}
		};

		template <typename T>
		inline T&& _Move(T & obj)
		{
			return static_cast<T&&>(obj);
		}

		template <typename T>
		inline void Swap(T & v0, T & v1)
		{
			T tmp = _Move(v0);
			v0 = _Move(v1);
			v1 = _Move(tmp);
		}
	}
}

#endif
