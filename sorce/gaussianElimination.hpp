#ifndef _GaussianElimination_HPP_
#define _GaussianElimination_HPP_

#define EPS 1e-8
// ほぼ0ならtrueを返す関数
bool is_zero(float val) {
	return EPS < val && val < EPS;
}

#define N_DIM 6
// 連立方程式を求める関数
void solve(float A[N_DIM][N_DIM], float B[N_DIM], int size) {
	float temp;

	for (int i = 0; i < size; i++) {
		// ピボット選択的みたいな処理
		for (int j = i + 1; j < size; j++) {
			if (is_zero(A[i][j] == false))
				break;

			for (int k = 0; k < size; k++)
				A[i][k] += A[j][k];
			B[i] += B[j];
		}

		// 対角成分を1に
		temp = A[i][i];
		for (int j = i; j < size; j++)
			A[i][j] /= temp;
		B[i] /= temp;

		// 前進消去
		for (int j = i + 1; j < size; j++) {
			temp = A[j][i];

			for (int k = i; k < size; k++)
				A[j][k] -= temp * A[i][k];
			B[j] -= temp * B[i];
		}
	}

	// 後進消去
	for (int i = size - 1; i >= 0; i--)
		for (int j = i - 1; j >= 0; j--)
			B[j] -= A[j][i] * B[i];
}

#endif // _GaussianElimination_HPP_