
//------------------------------------------------------------------------------
//  three.Matrix拡張.
/*
Matrix3.prototype.toMatrixArray = function(m) {
  //  [3][3]の配列にマトリクスを格納する.
  var te = this.elements;

  m[0][0] = te[0];  m[0][1] = te[1];  m[0][2] = te[2];
  m[1][0] = te[3];  m[1][1] = te[4];  m[1][2] = te[5];
  m[2][0] = te[6];  m[2][1] = te[7];  m[2][2] = te[8];

  return this;
};
*/

//------------------------------------------------------------------------------
var MBFunc3D = function() {

  this.GRAVITY = 9.81;

  //  歪み対称マトリクスを求める(外積オペレータ).
  this.tilde = function(pos) {
    var x = pos[0][0];
    var y = pos[1][0];
    var z = pos[2][0];

    var t = [
      [  0, -z,  y ],
      [  z,  0, -x ],
      [ -y,  x,  0 ]
    ];

    return t;
  };

  //  回転マトリクスを求める.
  this.rotMatrix = function(fai, theta, psi) {
    var C = [ Math.cos(fai), Math.cos(theta), Math.cos(psi) ];
    var S = [ Math.sin(fai), Math.sin(theta), Math.sin(psi) ];

    var mat = [
      [ C[0]*C[2]-S[0]*C[1]*S[2], -C[0]*S[2]-S[0]*C[1]*C[2],  S[0]*S[1] ],
      [ S[0]*C[2]+C[0]*C[1]*S[2], -S[0]*S[2]+C[0]*C[1]*C[2], -C[0]*S[1] ],
      [ S[1]*S[2],                 S[1]*C[2],                 C[1] ]
    ];

    return mat;
  };

  //  オイラー角の時間微分を求めるためのマトリクス(式4.14)
  this.invGed = function(fai, theta, psi) {
    var C = [ Math.cos(fai), Math.cos(theta), Math.cos(psi) ];
    var S = [ Math.sin(fai), Math.sin(theta), Math.sin(psi) ];

    var mat = [
      [  S[2],       C[2],      0 ],
      [  S[1]*C[2], -S[1]*S[2], 0 ],
      [ -C[1]*S[2], -C[1]*C[2], S[1] ]
    ];

    if(Math.abs(S[1]) > 0.000001)
      numeric.mul(mat, 1.0/S[1]);

    return mat;
  };

  //  クォータニオンから時間微分を求める行列を作る
  this.makeDQuatMatrix = function(x, y, z, w) {
    var mat = [
      [  w, -z,  y ],       //  x
      [  z,  w, -x ],       //  y
      [ -y,  x,  w ],       //  z
      [ -x, -y, -z ]        //  w
    ];

    return mat;
  };

  //  回転行列を求める.
  this.makeRotMatrixX = function(theta) {
    var c = Math.cos(theta);
    var s = Math.sin(theta);

    var mat = [
        [ 1, 0,  0 ],
        [ 0, c, -s ],
        [ 0, s,  c ]
    ];
    return mat;
  };

  this.makeRotMatrixY = function(theta) {
    var c = Math.cos(theta);
    var s = Math.sin(theta);

    var mat = [
        [  c, 0, s ],
        [  0, 1, 0 ],
        [ -s, 0, c ],
    ];
    return mat;
  };

  this.makeRotMatrixZ = function(theta) {
    var c = Math.cos(theta);
    var s = Math.sin(theta);

    var mat = [
        [ c, -s, 0 ],
        [ s,  c, 0 ],
        [ 0,  0, 1 ],
    ];
    return mat;
  };

  //  回転角のリミッター
  this.limitRot = function(rotarr) {
    var PI2 = Math.PI * 2.0;
    for(var lp1 = 0; lp1 < rotarr.legth; lp1++) {
      while(rotarr[lp1] >  PI2) rotarr[lp1] -= PI2;
      while(rotarr[lp1] < -PI2) rotarr[lp1] += PI2;
    }

    return rotarr;
  };

  //  複数のマトリクスを掛ける
  this.mulMatrix = function(mtx_array) {
    var ret = mtx_array[0];

    for(var lp1 = 1; lp1 < mtx_array.length; lp1++) {
      ret = numeric.dot(ret, mtx_array[lp1]);
    }
    return ret;
  };

  //  クォータニオンからマトリクスへ変換
  this.QuatToMtx = function(q) {
    var x = q[0], y = q[1], z = q[2], w = q[3];

    var mtx = [
      [ y*y - z*z - w*w + x*x, 2.0*(y*z - w*x),       2.0*(w*y + x*z) ],
      [ 2.0*(y*z + w*x),       z*z - w*w - y*y + x*x, 2.0*(z*w - y*x) ],
      [ 2.0*(w*y - z*x),       2.0*(z*w + y*x),       w*w - y*y - z*z + x*x ],
    ];

    return mtx;
  };

  //	２次元配列の生成.
  this.createArray = function(dx, dy) {
    var arr = [];

    for(var y = 0; y < dy; y++) {
      arr[y] = [];
      for(var x = 0; x < dx; x++) {
        arr[y][x] = 0;
      }
    }
    return arr;
  };

  //  単位行列の生成.
  this.createUnitArray = function(dim) {
      var arr = [];

      for(var y = 0; y < dim; y++) {
        arr[y] = [];
        for(var x = 0; x < dim; x++) {
          arr[y][x] = (y != x)? 0.0 : 1.0;
        }
      }

      return arr;
  };

  //	２次元配列のコピー
  //  dst      : 書き出す行列
  //  dx, dy   : 書き出す行列の位置
  //  src      : コピー元の行列
  //  dmx, dmy : コピーする横縦の大きさ
	this.copyArray = function(dst, dx, dy, src, dmx, dmy) {
		//  行列のサイズを取得.
		var dmy = src.length;
		var dmx = src[0].length;

		//  １次元の場合.
		if(typeof dmx === "undefined") {
			for(var y = 0; y < dmy; y++) {
				dst[dy+y][dx] = src[y];
			}
			return dst;
		}

		for(var y = 0; y < dmy; y++) {
			for(var x = 0; x < dmx; x++) {
				dst[dy+y][dx+x] = src[y][x];
			}
    }
    return dst;
	}

	//	２次元配列を転置させてコピー.
	this.copyArrayT = function(dst, dx, dy, src, dmx, dmy) {
		//  行列のサイズを取得.
		var dmy = src.length;
		var dmx = src[0].length;

		for(var y = 0; y < dmy; y++) {
			for(var x = 0; x < dmx; x++) {
				dst[dy+x][dx+y] = src[y][x];
			}
		}
	}

  //  行から列に変換する
  this.makeColumnArray = function(arr, st, cnt) {
    var ret = [];

    for(var lp1 = 0; lp1 < cnt; lp1++) {
      ret[lp1] = [];
      ret[lp1][0] = arr[st+lp1];
    }

    return ret;
  };

  //  列から行に変換する
  this.makeRowArray = function(arr, st, cnt, column) {
    var ret = [];

    if(!column) column = 0;

    for(var lp1 = 0; lp1 < cnt; lp1++) {
      ret[lp1] = arr[st+lp1][column];
    }

    return ret;
  };

  //------------------------------------------------------------------------------
  //  オイラーパラメータから回転行列を作る
  this.EtoC = function(e) {
    var E1 = e[0][0];
    var E2 = e[1][0];
    var E3 = e[2][0];
    var E4 = e[3][0];

    var c = [
      [ E2*E2-E3*E3-E4*E4+E1*E1, (E2*E3-E4*E1)*2,          (E4*E2+E3*E1)*2 ],
      [(E2*E3+E4*E1)*2,           E3*E3-E4*E4-E2*E2+E1*E1, (E3*E4-E2*E1)*2 ],
      [(E4*E2-E3*E1)*2,          (E3*E4+E2*E1)*2,           E4*E4-E2*E2-E3*E3+E1*E1 ]
    ];

    return c;
  };

  //------------------------------------------------------------------------------
  //  オイラーパラメータからS行列を作る
  this.EtoS = function(e) {
    var E0 = e[0][0];
    var E1 = e[1][0];
    var E2 = e[2][0];
    var E3 = e[3][0];

    var s = [[ -E1,  E0,  E3, -E2 ],
             [ -E2, -E3,  E0,  E1 ],
             [ -E3,  E2, -E1,  E0 ]];

    return s;
  };

  //------------------------------------------------------------------------------
  //  ベクトルから外積オペレータを作る
  this.TILDE = function(vec) {
    var A1 = vec[0][0];
    var A2 = vec[1][0];
    var A3 = vec[2][0];
    TildeA = [[   0, -A3,  A2 ],
              [  A3,   0, -A1 ],
              [ -A2,  A1,   0 ]];

    return TildeA;
  };

  //------------------------------------------------------------------------------
  //  次の時刻のパラメータを計算する
  var calcNextParam = function(p, dp, dt) {
    var n = [];

    //  要素の数でループ
    for(var lp1 = 0; lp1 < p.length; lp1++) {
        n[lp1] = p[lp1] + dp[lp1]*dt;
    }

    return n;
  }

  var calcRKAnswer = function(k1, k2, k3, k4) {
    var a = [];

    //  要素の数でループ
    for(var lp1 = 0; lp1 < k1.length; lp1++) {
        a[lp1] = (k1[lp1] + 2*k2[lp1] + 2*k3[lp1] + k4[lp1]) / 6;
    }

    return a;
  }

  //  ルンゲクッタルーチン
  //  func  : 微分方程式
  //  param : [ p0, p1...] パラメータ
  //  dt    : ステップ時間
  this.RKSolve = function(func, param, dt) {
    var p0 = param;

    //  １段目
    var k1 = func(p0);

    //  ２段目
    var p1 = calcNextParam(p0, k1, dt/2);
    var k2 = func(p1);

    //  ３段目
    var p2 = calcNextParam(p0, k2, dt/2);
    var k3 = func(p2);

    //  ４段目
    var p3 = calcNextParam(p0, k3, dt);
    var k4 = func(p3);

    //  答えを求める
    var ans = calcRKAnswer(k1, k2, k3, k4);

    //  次のステップの値を返す
    return calcNextParam(p0, ans, dt);
  };

};
