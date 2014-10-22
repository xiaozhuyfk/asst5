#include "PerformanceCounter.h"
#ifdef WIN32
#include <Windows.h>
#endif

using namespace std::chrono;

namespace CoreLib
{
	namespace Diagnostics
	{
		TimePoint PerformanceCounter::Start()
		{
			return high_resolution_clock::now();
		}

		Duration PerformanceCounter::End(TimePoint counter)
		{
			return high_resolution_clock::now()-counter;
		}

		double PerformanceCounter::ToSeconds(Duration counter)
		{
			auto rs = duration_cast<duration<double>>(counter);
			return *(double*)&rs;
		}
	}
}
