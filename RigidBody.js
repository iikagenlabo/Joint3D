//  マルチボディライブラリ
var MB = new MBFunc3D();

class RigidBody {
    constructor() {
        this.position = new THREE.Vector3();
        this.velocity = new THREE.Vector3();
        this.accel = new THREE.Vector3();   //[0, 0, 0];
        this.rot = [0, 0, 0];
        this.omega = new THREE.Vector3();
        this.d_omega = new THREE.Vector3(); //[0, 0, 0];
        this.quaternion = new THREE.Quaternion();

        this.mass = 1.0;
        this.invMass;
        this.inertia = new THREE.Vector3(1, 1, 1);
        this.invI;
        this.i_mtx;
        this.inv_i;

        this.model = null;
        this.gravity = false; //true;

        //  円柱用（仮）
        this.radius = 1;
        this.Width = 1;
    }

    preCalcParameter() {
        this.invMass = 1/this.mass;
        this.invI = [
            1/this.inertia.x,
            1/this.inertia.y,
            1/this.inertia.z
        ];

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

    //  ループ実行前の処理
    preExec() {
        //  加速度をクリアする
        this.accel.set(0, 0, 0);
        this.d_omega.set(0, 0, 0);
    }

    //  表示モデルの生成
    createModel() {
        // //  仮の円柱モデル
        // var material = new THREE.MeshPhongMaterial({
        //     color: 0x4040ff,
        //     shading: THREE.FlatShading
        // });

        // var base = new THREE.Object3D();

        // //  T型の物体を作る
        // let radius = 0.2;
        // let length = 1.0;
        // var geom = new THREE.CylinderGeometry(radius, radius, length, 8);
        // geom.rotateZ(Math.PI/2);       //  90度回転
        // let cy0 = new THREE.Mesh(geom, material);
        // cy0.position.y = -0.4;
        // cy0.castShadow = true;
        // cy0.receiveShadow = true;
        // base.add(cy0);

        // let length2 = 0.8;
        // geom = new THREE.CylinderGeometry(radius, radius, length2, 8);
        // let cy1 = new THREE.Mesh(geom, material);
        // cy1.castShadow = true;
        // cy1.receiveShadow = true;
        // base.add(cy1);

        // this.model = base;

        // return base;
    }

    //  剛体に力をかける
    applyImpulse(force, w_pos) {
        //  並進加速度
        this.accel.add(force);
        //  ワールド座標系でのベクトルと外積を取ってトルクを求める
        let torque = new THREE.Vector3();
        torque.crossVectors(w_pos, force);
        //  角加速度を更新
        this.d_omega.add(torque);
    }

    //  ローカルの点のワールド座標での速度を求める
    getLocalPointVelocity(l_pos) {
		var p_vel = this.omega.clone();
        p_vel.cross(l_pos);
        p_vel.add(this.velocity);

        return p_vel;
    }

    //  クォータニオンの掛け算
    crossQuaternion(a, b) {
        let q = new THREE.Quaternion();

        q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
        q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y
        q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x
        q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w

        return q;
    }

    //  コリオリ力を求める
    calcCoriolisForce(omega) {
        //  角速度の外積オペレータ
        let om = [[omega.x], [omega.y], [omega.z]];
        let tilde_omega = MB.tilde(om);

        //  角加速度の計算
        //  コリオリ力(-tw * J * w)
        let torque = math.multiply(tilde_omega, math.multiply(this.i_mtx, om))
        torque = math.multiply(torque, -1);

        return torque;  //  [[x], [y], [z]]
    }

    //  クォータニオンの正規化
    normalizeQuaternion(q) {
        let len = Math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
        if (len === 0) {
            return new THREE.Quaternion(0, 0, 0, 1);
        } else {
            len = 1 / len;
            return new THREE.Quaternion(q.x*len, q.y*len, q.z*len, q.w*len);
        }
    }

    updateRigidBody(param) {
        let ret = [];
        //  パラメータを展開
        let position = new THREE.Vector3(param[0], param[1], param[2]);
        let velocity = new THREE.Vector3(param[3], param[4], param[5]);
        let quaternion = new THREE.Quaternion(param[6], param[7], param[8], param[9]);
        let omega = new THREE.Vector3(param[10], param[11], param[12]);

        //  クォータニオンを正規化
        quaternion.normalize();

        //  加速度
        let accel = this.accel.clone();
        if(this.gravity) {
            //  重力をローカル座標系に変換して足す
            // let lg = new THREE.Vector3(0, -this.mass * MB.GRAVITY, 0);
            // let wlq = this.quaternion.clone();
            // wlq.inverse();
            // lg.applyQuaternion(wlq);
            // accel.add(lg);

            accel.y -= this.mass * MB.GRAVITY;
        }

        //  角速度の外積オペレータ
        // let om = [[omega.x], [omega.y], [omega.z]];
        // let tilde_omega = MB.tilde(om);

        // //  角加速度の計算
        // //  コリオリ力(-tw * J * w)
        // let torque = math.multiply(tilde_omega, math.multiply(this.i_mtx, om))
        // torque = math.multiply(torque, -1);
        // let d_omega = math.multiply(this.inv_i, torque);

        //  コリオリ力によるトルク
        let torque = this.calcCoriolisForce(omega);
        let d_omega = math.multiply(this.inv_i, torque);    //  慣性モーメントで割って角加速度にする
        d_omega[0][0] += this.d_omega.x;
        d_omega[1][0] += this.d_omega.y;
        d_omega[2][0] += this.d_omega.z;

        //  角速度からクォータニオンの時間微分を求める(dq = 1/2q*wv)
        let vec_qw = new THREE.Quaternion(omega.x, omega.y, omega.z, 0);
        let vec_dq = this.crossQuaternion(quaternion, vec_qw);
        vec_dq.x *= 0.5;
        vec_dq.y *= 0.5;
        vec_dq.z *= 0.5;
        vec_dq.w *= 0.5;

        //  結果を配列に詰めて返す
        //  速度、加速度、クォータニオンの時間微分、角加速度
        ret[0] = velocity.x;
        ret[1] = velocity.y;
        ret[2] = velocity.z;
        ret[3] = accel.x;
        ret[4] = accel.y;
        ret[5] = accel.z;
        ret[6] = vec_dq.x;
        ret[7] = vec_dq.y;
        ret[8] = vec_dq.z;
        ret[9] = vec_dq.w;
        ret[10] = d_omega[0][0];
        ret[11] = d_omega[1][0];
        ret[12] = d_omega[2][0];

        return ret;
    }

    calcNextParam(p, dp, dt) {
        var n = [];

        //  要素の数でループ
        for(var lp1 = 0; lp1 < p.length; lp1++) {
            n[lp1] = p[lp1] + dp[lp1]*dt;
        }

        return n;
      }

    //  パラメータを配列に詰める
    setArrayFromParameter() {
        let param = [];
        param[0] = this.position.x;
        param[1] = this.position.y;
        param[2] = this.position.z;
        param[3] = this.velocity.x;
        param[4] = this.velocity.y;
        param[5] = this.velocity.z;
        param[6] = this.quaternion.x;
        param[7] = this.quaternion.y;
        param[8] = this.quaternion.z;
        param[9] = this.quaternion.w;
        param[10] = this.omega.x;
        param[11] = this.omega.y;
        param[12] = this.omega.z;

        return param;
    }

    unpackArrayToParameter(param) {
        this.position.set(param[0], param[1], param[2]);
        this.velocity.set(param[3], param[4], param[5]);
        this.quaternion.set(param[6], param[7], param[8], param[9]);
        this.omega.set(param[10], param[11], param[12]);
    }

    exec(delta_t) {
        let param = this.setArrayFromParameter();

        // let ret = this.updateRigidBody(param);
        // let np = this.calcNextParam(param, ret, delta_t);

        //  thisを固定したメソッドを生成
        const func = this.updateRigidBody.bind(this);

        //  ルンゲ・クッタで次のフレームの位置を求める
        let np = MB.RKSolve(func, param, delta_t);

        //  計算結果を戻す
        this.unpackArrayToParameter(np);
    }

    exec_normal(delta_t) {
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
        this.quaternion.normalize();
        // this.quaternion = this.normalizeQuaternion(this.quaternion);
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

//  いらないところを整理しないといけないな

//  https://qiita.com/GANTZ/items/8a9d52c91cce902b44c9

