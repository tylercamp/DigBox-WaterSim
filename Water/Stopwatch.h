
#ifndef _STOPWATCH_H_
#define _STOPWATCH_H_

#include <Windows.h>
#include <cstdlib>

class Stopwatch
{
public:
	Stopwatch( )
	{
		reset( );
	}

	void reset( )
	{
		QueryPerformanceCounter( &m_Counter );
	}

	//	Elapsed time in ms
	inline float elapsedTime( )
	{
		LARGE_INTEGER currentTime;
		QueryPerformanceCounter( &currentTime );
		long elapsedTime = currentTime.QuadPart - m_Counter.QuadPart;

		LARGE_INTEGER freq;
		QueryPerformanceFrequency( &freq );

		double seconds = (double)elapsedTime / (double)freq.QuadPart;

		return (float)seconds;
	}

private:
	LARGE_INTEGER m_Counter;
};

#endif