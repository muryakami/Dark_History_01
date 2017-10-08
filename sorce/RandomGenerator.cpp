#include "RandomGenerator.h"

/*class random_generator {
public:
	random_generator() :
		x(123456789),
		y(362436069),
		z(521288629),
		w(88675123)

	{}

	random_generator(uint32_t Seed) {
		setSeed(Seed);
	}

	void setSeed(uint32_t Seed) {
		x = 1812433253U * (Seed ^ (Seed >> 30));
		y = 1812433253U * (x ^ (x >> 30)) + 1;
		z = 1812433253U * (y ^ (y >> 30)) + 2;
		w = 1812433253U * (z ^ (z >> 30)) + 3;
	}

	uint32_t operator()(void) {
		uint32_t t = (x ^ (x << 11));
		x = y;
		y = z;
		z = w;
		return (w = (w ^ (w >> 19)) ^ (t ^ (t >> 8)));
	}

	int32_t operator()(int32_t Min, int32_t Max) {
		uint32_t t = (x ^ (x << 11));
		x = y;
		y = z;
		z = w;
		w = (w ^ (w >> 19)) ^ (t ^ (t >> 8));
		return ((int32_t)(w % (Max - Min + 1)) + Min);
	}

	uint32_t x, y, z, w;
};*/


random_generator::random_generator() :
	x(123456789),
	y(362436069),
	z(521288629),
	w(88675123)

{}

random_generator::random_generator(uint32_t Seed) {
	setSeed(Seed);
}

void random_generator::setSeed(uint32_t Seed) {
	x = 1812433253U * (Seed ^ (Seed >> 30));
	y = 1812433253U * (x ^ (x >> 30)) + 1;
	z = 1812433253U * (y ^ (y >> 30)) + 2;
	w = 1812433253U * (z ^ (z >> 30)) + 3;
}

uint32_t random_generator::operator()(void) {
	uint32_t t = (x ^ (x << 11));
	x = y;
	y = z;
	z = w;
	return (w = (w ^ (w >> 19)) ^ (t ^ (t >> 8)));
}

int32_t random_generator::operator()(int32_t Min, int32_t Max) {
	uint32_t t = (x ^ (x << 11));
	x = y;
	y = z;
	z = w;
	w = (w ^ (w >> 19)) ^ (t ^ (t >> 8));
	return ((int32_t)(w % (Max - Min + 1)) + Min);
}