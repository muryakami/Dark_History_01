#ifndef _GaussianElimination_HPP_
#define _GaussianElimination_HPP_

#define EPS 1e-8
// �ق�0�Ȃ�true��Ԃ��֐�
bool is_zero(float val) {
	return EPS < val && val < EPS;
}

#define N_DIM 6
// �A�������������߂�֐�
void solve(float A[N_DIM][N_DIM], float B[N_DIM], int size) {
	float temp;

	for (int i = 0; i < size; i++) {
		// �s�{�b�g�I��I�݂����ȏ���
		for (int j = i + 1; j < size; j++) {
			if (is_zero(A[i][j] == false))
				break;

			for (int k = 0; k < size; k++)
				A[i][k] += A[j][k];
			B[i] += B[j];
		}

		// �Ίp������1��
		temp = A[i][i];
		for (int j = i; j < size; j++)
			A[i][j] /= temp;
		B[i] /= temp;

		// �O�i����
		for (int j = i + 1; j < size; j++) {
			temp = A[j][i];

			for (int k = i; k < size; k++)
				A[j][k] -= temp * A[i][k];
			B[j] -= temp * B[i];
		}
	}

	// ��i����
	for (int i = size - 1; i >= 0; i--)
		for (int j = i - 1; j >= 0; j--)
			B[j] -= A[j][i] * B[i];
}

#endif // _GaussianElimination_HPP_