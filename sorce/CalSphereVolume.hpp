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
	cout << "�p�x: " << degree << "\tsin: " << s << "\tcos: " << c << endl;
	cout << "�~��: " << cone << endl;
	cout << "����: " << sagittal << endl;
	cout << "�̐�1: " << cone + sagittal << endl;
	cout << "�̐�2: " << volume << endl;
}

void calSphereSurface(double degree) {
	double rad = degree * M_PI / 180;
	double s = sin(rad);
	double c = abs(cos(rad));
	double surface = M_PI*(2 * (1 - c) + s);
	double cector = M_PI * s;
	double cap = 2.0f * M_PI*(1 - c);

	cout << endl;
	cout << "�p�x: " << degree << "\tsin: " << s << "\tcos: " << c << endl;
	cout << "����: " << surface << endl;
	cout << "��`: " << cector << endl;
	cout << "�\�ʐ�1: " << surface - cector << endl;
	cout << "�\�ʐ�2: " << cap << endl;
}

#endif  _CalSphereVolume_HPP_