
//  反復計算が収束したかどうか
function checkError(e) {
	const ERROR_VAL = 1.0e-10;

	for (var i = 0; i < e.length; i++) {
		//  値が収束したら反復終了
		if (e[i] <= ERROR_VAL) {
			return false;
		}
	}
	return true;
}

//  ガウスザイデル法で連立方程式を解く
//  Ax = b の xを求める
function GaussSeidel(a_mtx, b_vec, x_vec) {
	let N = b_vec.length;		//  求める変数の数
	var sum, val;
	var e = [];

	do {
		//  行でループ
		for (var i = 0; i < N; i++) {
			sum = 0.0;

			//  要素数でループ
			for (var j = 0; j < N; j++) {
				//  求める変数以外の場合
				if (i != j) {
					sum += a_mtx[i][j] * x_vec[j];
				}
			}

			//  求める答え
			val = (1.0 / a_mtx[i][i]) * (b_vec[i] - sum);
			//  前回との差分を保存
			e[i] = Math.abs(val - x_vec[i]);
			//  答えを格納
			x_vec[i] = val;
		}
	} while (checkError(e));

	return x_vec;
}
