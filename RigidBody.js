//  マルチボディライブラリ
var MB = new MBFunc3D();

class RigidBody {
    constructor() {
        this.position = new THREE.Vector3();
        this.velocity = new THREE.Vector3();
        this.accel = [0, 0, 0];
        this.rot = [0, 0, 0];
        this.omega = new THREE.Vector3();
        this.d_omega = [0, 0, 0];
        this.quaternion = new THREE.Quaternion();

        this.mass = 1.0;
        this.inertia = new THREE.Vector3(1, 1, 1);

        this.model = null;

        //  円柱用（仮）
        this.radius = 1;
        this.Width = 1;
    }

    preCalcParameter() {
        //  慣性モーメントを入れておく
        // this.inertia.x = 0.13;
        // this.inertia.y = 0.056;
        // this.inertia.z = 0.172;
        this.inertia.x = 1;
        this.inertia.y = 2;
        this.inertia.z = 4;

        this.i_mtx = [
            [this.inertia.x, 0, 0],
            [0, this.inertia.y, 0],
            [0, 0, this.inertia.z]
        ];
        this.inv_i = [
            [1/this.inertia.x, 0, 0],
            [0, 1/this.inertia.y, 0],
            [0, 0, 1/this.inertia.z]
        ];
    }

    createModel() {
        //  仮の円柱モデル
        var material = new THREE.MeshPhongMaterial({
            color: 0x4040ff,
            shading: THREE.FlatShading
        });

        var base = new THREE.Object3D();

        //  T型の物体を作る
        let radius = 0.2;
        let length = 1.0;
        var geom = new THREE.CylinderGeometry(radius, radius, length, 8);
        geom.rotateZ(Math.PI/2);       //  90度回転
        let cy0 = new THREE.Mesh(geom, material);
        cy0.position.y = -0.4;
        cy0.castShadow = true;
        cy0.receiveShadow = true;
        base.add(cy0);

        let length2 = 0.8;
        geom = new THREE.CylinderGeometry(radius, radius, length2, 8);
        let cy1 = new THREE.Mesh(geom, material);
        cy1.castShadow = true;
        cy1.receiveShadow = true;
        base.add(cy1);

        this.model = base;

        return base;
    }

    crossQuaternion(a, b) {
        let q = new THREE.Quaternion();

        q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
        q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y
        q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x
        q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w

        return q;
    }

    normalizeQuaternion(q) {
        let len = Math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        if (len === 0) {
            return new THREE.Quaternion(0, 0, 0, 1);
        } else {
            len = 1 / len;
            return new THREE.Quaternion(q.x*len, q.y*len, q.z*len, q.w*len);
        }
    }

    exec(delta_t) {
        // let q = this.quaternion;
        // let w = this.omega;
        // console.log('Q:', q.w, q.x, q.y, q.z);
        // console.log('W:', w.x, w.y, w.z);

        //  位置を更新
        var dvel = this.velocity.clone();
        dvel.multiplyScalar(delta_t);
        this.position.add(dvel);

        //  角速度の外積オペレータ
        let om = [[this.omega.x], [this.omega.y], [this.omega.z]];
        let tilde_omega = MB.tilde(om);

        //  角加速度の計算
        let torque = math.multiply(tilde_omega, math.multiply(this.i_mtx, om))
        torque = math.multiply(torque, -1);
        let d_omega = math.multiply(this.inv_i, torque);

        //  角速度を更新
        d_omega = math.multiply(d_omega, delta_t);
        this.omega.x += d_omega[0][0];
        this.omega.y += d_omega[1][0];
        this.omega.z += d_omega[2][0];

        //  角速度からクォータニオンの時間微分を求める(dq = 1/2q*wv)
        let vec_qw = new THREE.Quaternion(this.omega.x, this.omega.y, this.omega.z, 0);
        let vec_dq = this.quaternion.clone();
        // vec_dq.multiply(vec_qw);
        vec_dq = this.crossQuaternion(vec_dq, vec_qw);
        vec_dq.x *= 0.5 * delta_t;
        vec_dq.y *= 0.5 * delta_t;
        vec_dq.z *= 0.5 * delta_t;
        vec_dq.w *= 0.5 * delta_t;

        //  クォータニオンの更新
        this.quaternion.x += vec_dq.x;
        this.quaternion.y += vec_dq.y;
        this.quaternion.z += vec_dq.z;
        this.quaternion.w += vec_dq.w;
        // this.quaternion.normalize();
        this.quaternion = this.normalizeQuaternion(this.quaternion);

   
    // A=np.array([[q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3]  ],
    //             [q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2]  ],
    //             [q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1]  ],
    //             [q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0]  ]])

        // w =  aw*bw - ax*bx - ay*by - az*bz
        // x =  aw*bx + ax*bw + ay*bz - az*by
        // y =  aw*by - ax*bz + ay*bw + az*bx
        // z =  aw*bz + ax*by - ay*bx + az*bw

		// this._w = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;
		// this._x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
		// this._y = qay * qbw + qaw * qby + qaz * qbx - qax * qbz;
		// this._z = qaz * qbw + qaw * qbz + qax * qby - qay * qbx;


    }

    exec_test(delta_t) {
        //  回転運動のトルクを求める
        let om = [[this.omega.x], [this.omega.y], [this.omega.z]];
        let tilde_omega = MB.tilde(om);

        let torque = math.multiply(math.multiply(tilde_omega, this.i_mtx), om);
        torque = math.multiply(torque, -1);
        let d_omega = math.multiply(this.inv_i, torque);

        d_omega = math.multiply(d_omega, delta_t);
        this.omega.x += d_omega[0][0];
        this.omega.y += d_omega[1][0];
        this.omega.z += d_omega[2][0];

        //  位置を更新
        var dvel = this.velocity.clone();
        dvel.multiplyScalar(delta_t);
        this.position.add(dvel);

        //  角速度からクォータニオンの時間微分を計算する
        let omega = this.omega.clone();
        let qDot = new THREE.Quaternion().setFromAxisAngle(omega.normalize(), this.omega.length()/2).multiply(this.quaternion);
        // qDot.x = qDot.x * 0.5 * delta_t;
        // qDot.y = qDot.y * 0.5 * delta_t;
        // qDot.z = qDot.z * 0.5 * delta_t;
        // qDot.w = qDot.w * 0.5 * delta_t;
        qDot.x = qDot.x * 0.5;
        qDot.y = qDot.y * 0.5;
        qDot.z = qDot.z * 0.5;
        qDot.w = qDot.w * 0.5;

        //  オイラ―パラメータからS行列を求める
        // let S = MB.EtoS(this.quatToArray());
        //  角速度からオイラ―パラメータの時間微分を求める ST * Omega /2
        // let dE = math.multiply(math.transpose(S), this.omegaToArray(delta_t*0.5));  //  全体を２で割る

        //  角速度からオイラ―パラメータの時間微分を求める
        let dq_mtx = this.omegaToDQuatMatrix(delta_t*0.5);
        let dE = math.multiply(dq_mtx, this.quatToArray());
        // dE = math.multiply(dE, 0.5 * delta_t);

        //  クォータニオンの時間微分
        //  https://qiita.com/GANTZ/items/8a9d52c91cce902b44c9
        // let vec_qw = [[0], [this.omega.x], [this.omega.y], [this.omega.z]];
        let vec_qw = new THREE.Quaternion(this.omega.x, this.omega.y, this.omega.z, 0);
        let vec_dq = this.quaternion.clone();
        // vec_dq.multiply(vec_qw);
        // vec_dq.x *= 0.5 * delta_t;
        // vec_dq.y *= 0.5 * delta_t;
        // vec_dq.z *= 0.5 * delta_t;
        // vec_dq.w *= 0.5 * delta_t;
        vec_qw.multiply(vec_dq);
        vec_qw.x *= 0.5 * delta_t;
        vec_qw.y *= 0.5 * delta_t;
        vec_qw.z *= 0.5 * delta_t;
        vec_qw.w *= 0.5 * delta_t;

        // console.log(dE[0][0], dE[1][0], dE[2][0], dE[3][0]);

        //  回転角を更新
        let q = this.quaternion.clone();
        // let dq = new THREE.Quaternion(dE[0][0], dE[1][0], dE[2][0], dE[3][0]);
        q.x += vec_qw.x;
        q.y += vec_qw.y;
        q.z += vec_qw.z;
        q.w += vec_qw.w;
        // q.x += vec_dq.x;
        // q.y += vec_dq.y;
        // q.z += vec_dq.z;
        // q.w += vec_dq.w;
        // q.x += dE[0][0];
        // q.y += dE[1][0];
        // q.z += dE[2][0];
        // q.w += dE[3][0];
        // q.multiply(dq, q);

        // q.x += qDot.x;
        // q.y += qDot.y;
        // q.z += qDot.z;
        // q.w += qDot.w;

        q.normalize();
        this.quaternion.copy(q);

        //  回転角を更新する
        // let dq = new THREE.Quaternion();
        // dq.set(dE[1][0], dE[2][0], dE[3][0], dE[0][0]);
        // //  試しに回転
        // // dq.setFromEuler(new THREE.Euler(0.01, 0, 0));

        // // dq.multiply(this.quaternion);
        // // dq.normalize();
        // dq.x += this.quaternion.x;
        // dq.y += this.quaternion.y;
        // dq.z += this.quaternion.z;
        // dq.w += this.quaternion.w;
        // dq.normalize();

        // this.quaternion.copy(dq);
    }

    //  位置と回転角をモデルに反映
    updatePosRot() {
        if (this.model == null) return;

        //  モデルを回転させる.
        this.model.quaternion.copy(this.quaternion);
        //  位置の設定
        this.model.position.copy(this.position);
    }

    //  THREE.Quaternion をオイラ―パラメータの配列に変換
    quatToArray() {
        return [
            [this.quaternion.x],
            [this.quaternion.y],
            [this.quaternion.z],
            [this.quaternion.w]
        ];
    }

    omegaToArray(dt = 1) {
        return [
            [this.omega.x * dt],
            [this.omega.y * dt],
            [this.omega.z * dt]
        ];
    }

    omegaToDQuatMatrix(dt = 1) {
        let om = this.omega.clone();
        om.multiplyScalar(dt);

        //  位置ベクトルの回転(x, y, z, w)
        // return [
        //     [0,     -om.z,  om.y,  om.x],
        //     [ om.z,     0, -om.x,  om.y],
        //     [-om.y,  om.x,     0,  om.z],
        //     [-om.x, -om.y, -om.z,     0]
        // ];
        //  (w, x, y, z)
        return [
            [0,     -om.x, -om.y, -om.z],
            [ om.x,     0, -om.z,  om.y],
            [ om.y,  om.z,     0, -om.x],
            [ om.z, -om.y,  om.x,     0]
        ];

        //  座標系の回転
        // return [
        //     [0,      om.z, -om.y,  om.x],
        //     [-om.z,     0,  om.x,  om.y],
        //     [ om.y, -om.x,     0,  om.z],
        //     [-om.x, -om.y, -om.z,     0]
        // ];
    }
}



//  https://qiita.com/GANTZ/items/8a9d52c91cce902b44c9

