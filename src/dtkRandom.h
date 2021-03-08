#ifndef DTK_RANDOM_H
#define DTK_RANDOM_H

namespace dtk
{
	//Get a random int x which between a <= x < b
	inline int dtkRandomInt(int a, int b)
	{
		int r = std::rand() % (b - a);
		return a + r;
	}

	//Generate a random float in [0.0, 1.0]
	inline float dtkRandomFloat()
	{
		return ((float)rand()) / ((float)RAND_MAX);
	}

	//Generate a random double in [0.0, 1.0]
	inline double dtkRandomDouble()
	{
		return ((double)rand()) / ((double)RAND_MAX);
	}
}

#endif //DTK_RANDOM_H
