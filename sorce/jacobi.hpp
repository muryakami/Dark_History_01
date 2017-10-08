#ifndef _Jacobi_HPP_
#define _Jacobi_HPP_

int Jacobi(int n, int ct, double eps, double **A, double **A1, double **A2, double **X1, double **X2) {
	double max, s, t, v, sn, cs;
	int i1, i2, k = 0, ind = 1, p = 0, q = 0;
	// 初期設定
	for (i1 = 0; i1 < n; i1++) {
		for (i2 = 0; i2 < n; i2++) {
			A1[i1][i2] = A[i1][i2];
			X1[i1][i2] = 0.0;
		}
		X1[i1][i1] = 1.0;
	}
	// 計算
	while (ind > 0 && k < ct) {
		// 最大要素の探索
		max = 0.0;
		for (i1 = 0; i1 < n; i1++) {
			for (i2 = 0; i2 < n; i2++) {
				if (i2 != i1) {
					if (fabs(A1[i1][i2]) > max) {
						max = fabs(A1[i1][i2]);
						p = i1;
						q = i2;
					}
				}
			}
		}
		// 収束判定
		// 収束した
		if (max < eps)
			ind = 0;
		// 収束しない
		else {
			// 準備
			s = -A1[p][q];
			t = 0.5 * (A1[p][p] - A1[q][q]);
			v = fabs(t) / sqrt(s * s + t * t);
			sn = sqrt(0.5 * (1.0 - v));
			if (s*t < 0.0)
				sn = -sn;
			cs = sqrt(1.0 - sn * sn);
			// Akの計算
			for (i1 = 0; i1 < n; i1++) {
				if (i1 == p) {
					for (i2 = 0; i2 < n; i2++) {
						if (i2 == p)
							A2[p][p] = A1[p][p] * cs * cs + A1[q][q] * sn * sn -
							2.0 * A1[p][q] * sn * cs;
						else if (i2 == q)
							A2[p][q] = 0.0;
						else
							A2[p][i2] = A1[p][i2] * cs - A1[q][i2] * sn;
					}
				}
				else if (i1 == q) {
					for (i2 = 0; i2 < n; i2++) {
						if (i2 == q)
							A2[q][q] = A1[p][p] * sn * sn + A1[q][q] * cs * cs +
							2.0 * A1[p][q] * sn * cs;
						else if (i2 == p)
							A2[q][p] = 0.0;
						else
							A2[q][i2] = A1[q][i2] * cs + A1[p][i2] * sn;
					}
				}
				else {
					for (i2 = 0; i2 < n; i2++) {
						if (i2 == p)
							A2[i1][p] = A1[i1][p] * cs - A1[i1][q] * sn;
						else if (i2 == q)
							A2[i1][q] = A1[i1][q] * cs + A1[i1][p] * sn;
						else
							A2[i1][i2] = A1[i1][i2];
					}
				}
			}
			// Xkの計算
			for (i1 = 0; i1 < n; i1++) {
				for (i2 = 0; i2 < n; i2++) {
					if (i2 == p)
						X2[i1][p] = X1[i1][p] * cs - X1[i1][q] * sn;
					else if (i2 == q)
						X2[i1][q] = X1[i1][q] * cs + X1[i1][p] * sn;
					else
						X2[i1][i2] = X1[i1][i2];
				}
			}
			// 次のステップへ
			k++;
			for (i1 = 0; i1 < n; i1++) {
				for (i2 = 0; i2 < n; i2++) {
					A1[i1][i2] = A2[i1][i2];
					X1[i1][i2] = X2[i1][i2];
				}
			}
		}
	}

	return ind;
}

#endif // _Jacobi_HPP_