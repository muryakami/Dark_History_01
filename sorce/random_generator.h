#ifndef _RANDOMGENERATOR_H_
#define _RANDOMGENERATOR_H_

#include <cstdint>

class random_generator {
public:
	random_generator();

	random_generator(uint32_t);

	void setSeed(uint32_t);

	uint32_t operator()(void);

	int32_t operator()(int32_t, int32_t);

	uint32_t x, y, z, w;
};

#endif // _RANDOMGENERATOR_H_