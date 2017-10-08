#include "RandUnitVector.h"

void randUnitVector(vector3* pOut, random_generator& Rand) {
	uint32_t value = Rand();
	uint32_t theta;
	float sin, cos;
	// pitch
	theta = (value & 0xFFFF0000);
	theta = (theta >> 9) | (theta >> 25) | 0x40000000;
	//sincos((*(float*)&theta - 3.0f) * M_PI, &sin, &cos);
	//sincosf((*(float*)&theta - 3.0f) * M_PI, &sin, &cos);
	sin = sinf((*(float*)&theta - 3.0f) * PI);
	cos = cosf((*(float*)&theta - 3.0f) * PI);
	pOut->x = cos;
	pOut->y = cos;
	pOut->z = -sin;
	// yaw
	theta = (value & 0x0000FFFF);
	theta = (theta << 7) | (theta >> 9) | 0x40000000;
	//sincos((*(float*)&theta - 3.0f) * PI, &sin, &cos);
	sin = sinf((*(float*)&theta - 3.0f) * PI);
	cos = cosf((*(float*)&theta - 3.0f) * PI);
	pOut->x *= cos;
	pOut->y *= sin;
}