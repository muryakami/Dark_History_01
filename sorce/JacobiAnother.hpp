#ifndef _JacobiAnother_HPP_
#define _JacobiAnother_HPP_

#include <math.h>
#define   Jacobi_TOL     1.0e-10   
#define   Jacobi_N       2
#define   Jacobi_MAX     1000

int S_Jacobi(double *, double *, int);
void S_OutMat(double *, int, int);

int S_RunJacobi(void) {
	int    i, j, status;
	double x[Jacobi_N*Jacobi_N];

	//   	���̓f�[�^�i���4.2�j
	//double a[Jacobi_N*Jacobi_N] = { 10.0,3.0,2.0, 3.0,5.0,1.0, 2.0,1.0,0.0 };
	double a[Jacobi_N*Jacobi_N] = { 4.0,1.0, -2.0,1.0 };

	std::cout << "�s��" << std::endl;
	S_OutMat(a, Jacobi_N, Jacobi_N);

	printf(" Jacobi�@�ɂ��ŗL�l��� \n");
	status = S_Jacobi(a, x, Jacobi_N);

	//   ��͌��ʂ̏o��
	if (status == 0) {
		printf(" �ŗL�l \n");
		for (i = 0; i<Jacobi_N; i++) printf(" %10.6f", a[Jacobi_N*i + i]);
		printf("\n");
		printf(" �ŗL�x�N�g�� \n");
		S_OutMat(x, Jacobi_N, Jacobi_N);
	}
	return 0;
}

/*********  �ŗL�l��Jacobi�@�ɂ�鋁�߂镛�֐�  ************************/
/*   �߂�l�i�G���[�R�[�h�j�F  0=����,      9: �ُ�                    */
/*   ���́F a[n�~n]=�s��C  n: �v�f��                                  */
/*   �o�́F �ŗL�l�i���j�� a �̑Ίp�v�f                                */
/*          �ŗL�x�N�g���� x[n�~n]                                     */
/************************************************************************/
int S_Jacobi(double a[], double x[], int n)
{
	int    i, j, k, m, count, status;
	double amax, amax0, theta, co, si, co2, si2, cosi, pi = 4.0*atan(1.0);
	double aii, aij, ajj, aik, ajk;

	//   �����l�ݒ�
	for (i = 0; i<n; i++) {
		for (j = 0; j<n; j++) {
			if (i == j)  x[n*i + j] = 1.0; else  x[n*i + j] = 0.0;
		}
	}

	//   �����v�Z
	count = 0;  status = 9;
	while (count <= Jacobi_MAX) {
		//  ��Ίp�v�f�̍ő�l��T��
		amax = 0.0;
		for (k = 0; k<n - 1; k++) {
			for (m = k + 1; m<n; m++) {
				amax0 = fabs(a[n*k + m]);
				if (amax0 > amax) { i = k;  j = m, amax = amax0; }
			}
		}
		//  ��������
		if (amax <= Jacobi_TOL) { status = 0;   break; }
		else {
			aii = a[n*i + i];   aij = a[n*i + j];   ajj = a[n*j + j];
			//   ��]�p�x�v�Z
			if (fabs(aii - ajj) < Jacobi_TOL) {
				theta = 0.25*pi*aij / fabs(aij);
			}
			else {
				theta = 0.5*atan(2.0*aij / (aii - ajj));
			}
			co = cos(theta); si = sin(theta); co2 = co*co; si2 = si*si; cosi = co*si;

			//   �����ϊ��s��
			a[n*i + i] = co2*aii + 2.0*cosi*aij + si2*ajj;
			a[n*j + j] = si2*aii - 2.0*cosi*aij + co2*ajj;
			a[n*i + j] = 0.0;    a[n*j + i] = 0.0;
			for (k = 0; k<n; k++) {
				if (k != i && k != j) {
					aik = a[n*k + i];            ajk = a[n*k + j];
					a[n*k + i] = co*aik + si*ajk;  a[n*i + k] = a[n*k + i];
					a[n*k + j] = -si*aik + co*ajk;  a[n*j + k] = a[n*k + j];
				}
			}

			//   �ŗL�x�N�g��
			for (k = 0; k<n; k++) {
				aik = x[n*k + i];   ajk = x[n*k + j];
				x[n*k + i] = co*aik + si*ajk;
				x[n*k + j] = -si*aik + co*ajk;
			}
			count++;
		}
		printf(" S_Jacobi> iter=%d", count);
		for (i = 0; i<n; i++)  printf(" %10.6f,", a[n*i + i]);
		printf("\n");
	}
	return status;
}
/*********  �s���\�����镛�֐�  *******************************/
/*       ���́G   a= �s��, n= �\�������, m= �\������s��     */
/****************************************************************/
void S_OutMat(double a[], int n, int m)
{
	int i, j;
	for (i = 0; i<n; i++) {
		for (j = 0; j<m; j++)  printf("  %10.6f", a[i*m + j]);
		printf("\n");
	}
}

#endif // _JacobiAnother_HPP_