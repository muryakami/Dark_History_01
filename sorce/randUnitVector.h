#ifndef _RUV_H_
#define _RUV_H_

#include "RandomGenerator.h"

#include <amp_math.h>
//using namespace Concurrency::precise_math;
//using namespace Concurrency::fast_math;

//#define _USE_MATH_DEFINES
//#include <math.h>
#define PI 3.141592654f

struct vector3 {
	float x;
	float y;
	float z;
};

void randUnitVector(vector3*, random_generator&);

#endif // _RUV_H_