#ifndef _CalSphereVolume_HPP_
#define _CalSphereVolume_HPP_

#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
using namespace std;

void calSphereVolume(double degree) {
	double rad = degree * M_PI / 180;
	double s = sin(rad);
	double c = abs(cos(rad));
	double cone = s * s * M_PI * c / 3;
	double sagittal = M_PI / 6 * (1 - c) * (3*s*s + ((1-c)*(1-c)));
	double volume = 2.0f / 3.0f * M_PI*(1 - c);

	cout << endl;
	cout << "Šp“x: " << degree << "\tsin: " << s << "\tcos: " << c << endl;
	cout << "‰~: " << cone << endl;
	cout << "‹…Œ‡: " << sagittal << endl;
	cout << "‘ÌÏ1: " << cone + sagittal << endl;
	cout << "‘ÌÏ2: " << volume << endl;
}

void calSphereSurface(double degree) {
	double rad = degree * M_PI / 180;
	double s = sin(rad);
	double c = abs(cos(rad));
	double surface = M_PI*(2 * (1 - c) + s);
	double cector = M_PI * s;
	double cap = 2.0f * M_PI*(1 - c);

	cout << endl;
	cout << "Šp“x: " << degree << "\tsin: " << s << "\tcos: " << c << endl;
	cout << "‹…•ª: " << surface << endl;
	cout << "îŒ`: " << cector << endl;
	cout << "•\–ÊÏ1: " << surface - cector << endl;
	cout << "•\–ÊÏ2: " << cap << endl;
}

#endif  _CalSphereVolume_HPP_