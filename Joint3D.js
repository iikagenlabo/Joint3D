class Joint3D {
    static Gravity = 9.81;
    static Sqrt12 = 0.7071067811865475244008443621048490;

    constructor(body_a, lp_a, body_b, lp_b) {
        //  剛体の番号
        this.body_a = body_a;
        this.body_b = body_b;

        //  剛体内でのジョイントの位置(Vector3)
        this.lp_a = lp_a;
        this.lp_b = lp_b;
        this.wp_a;
        this.wp_b;

        this.J = [];        //  ヤコビアン
        this.JT = [];       //  ヤコビアンの転置
        this.invM = [];     //  質量の逆数

        this.Amtx = [];

        //  拘束力
        this.constraintForce = new THREE.Vector3();
        this.constraintTorque = new THREE.Vector3();

        //  拘束補正値
        this.p_err = new THREE.Vector3();   //  位置差分
        this.v_err = new THREE.Vector3();   //  速度差分
    }

    //  事前計算
    preCalc(delta_t) {
        //  拘束点のワールド座標を求める準備
        this.wp_a = new THREE.Vector3(this.lp_a.x, this.lp_a.y, this.lp_a.z);
        this.wp_b = new THREE.Vector3(this.lp_b.x, this.lp_b.y, this.lp_b.z);
        let invMa = 0;
        let invMb = 0;
        let invIa = [0, 0, 0];
        let invIb = [0, 0, 0];

        if (this.body_a != null && !this.body_a.isWorldBody()) {
            this.wp_a.applyQuaternion(this.body_a.getQuaternion());
            invMa = this.body_a.invMass;
            invIa = this.body_a.invI;
        }
        if (this.body_b != null && !this.body_b.isWorldBody()) {
            this.wp_b.applyQuaternion(this.body_b.getQuaternion());
            invMb = this.body_b.invMass;
            invIb = this.body_b.invI;
        }

        //  質量マトリクスの逆数[12*12]
        this.invM = [
            [invMa, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, invMa, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, invMa, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, invIa[0], 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, invIa[1], 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, invIa[2], 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, invMb, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, invMb, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, invMb, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, invIb[0], 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, invIb[1], 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, invIb[2]]
        ];
    }

    //  拘束力の計算
    //  外力を加えて方程式の右辺を作って、連立方程式を解いて拘束力を求める
    calcConstraint(delta_t) {
        //  Amtx = [J][M-1][JT] は Joint3D で作る
        let Amtx = this.Amtx;

        //  エラー補正用の差分
        let err = this.p_err.clone();
        // err.add(this.v_err);

        //  速度
        let u = [[0], [0], [0], [0], [0], [0],
        [0], [0], [0], [0], [0], [0]];
        if (this.body_a != null && !this.body_a.isWorldBody()) {
            let vel = this.body_a.getVelocity().clone();
            vel.sub(err);
            u[0] = [vel.x - err.x];
            u[1] = [vel.y - err.y];
            u[2] = [vel.z - err.z];
            u[3] = [this.body_a.getOmega().x];
            u[4] = [this.body_a.getOmega().y];
            u[5] = [this.body_a.getOmega().z];
        }
        if (this.body_b != null && !this.body_b.isWorldBody()) {
            let vel = this.body_b.getVelocity().clone();
            vel.add(err);
            u[6] = [vel.x + err.x];
            u[7] = [vel.y + err.y];
            u[8] = [vel.z + err.z];
            u[9] = [this.body_b.getOmega().x];
            u[10] = [this.body_b.getOmega().y];
            u[11] = [this.body_b.getOmega().z];
        }

        //  外力
        let F = [[0], [0], [0], [0], [0], [0],
        [0], [0], [0], [0], [0], [0]];
        if (this.body_a != null && !this.body_a.isWorldBody()) {
            F[1] = [-this.body_a.mass * Joint3D.Gravity];   //  重力
            //  コリオリ力
            let torque = this.body_a.calcCoriolisForce(this.body_a.getOmega());
            F[3] = torque[0];
            F[4] = torque[1];
            F[5] = torque[2];
        }
        if (this.body_b != null && !this.body_b.isWorldBody()) {
            F[8] = [-this.body_b.mass * Joint3D.Gravity];   //  重力
            //  コリオリ力
            let torque = this.body_b.calcCoriolisForce(this.body_b.getOmega());
            F[9] = torque[0];
            F[10] = torque[1];
            F[11] = torque[2];
        }

        //  加速度にdtを掛けないので、速度をdtで割って単位をそろえる
        u = math.multiply(u, 1 / delta_t);

        //  Bvec = J(u + [M-1][F]*dt) [3*1]
        let Bvec = math.multiply(this.invM, F);
        // Bvec = math.dotMultiply(Bvec, delta_t);
        Bvec = math.add(Bvec, u);
        Bvec = math.multiply(this.J, Bvec);
        //  普通の配列に変換
        let Barr = [];
        for (var i = 0; i < Bvec.length; i++)
        {
            Barr.push(Bvec[i][0]);
        }
        let impulse = [0, 0, 0, 0, 0, 0];

        //  連立方程式を解く A[3*3]I[3*1] = B[3*1]
        GaussSeidel(Amtx, Barr, impulse);

        this.constraintForce.set(impulse[0], impulse[1], impulse[2]);
        this.constraintTorque.set(impulse[3], impulse[4], impulse[5]);

        return impulse;
    }

    //  剛体に拘束力を掛ける
    applyConstraintForce() {
        if (this.body_a != null && !this.body_a.isWorldBody()) {
            let cf = this.constraintForce.clone();
            cf.multiplyScalar(-1);
            this.body_a.applyImpulse(cf, this.lp_a);
        }
        if (this.body_b != null && !this.body_b.isWorldBody()) {
            this.body_b.applyImpulse(this.constraintForce, this.lp_b);
        }
    }

    //  ジョイント位置のワールド座標
    getWorldPosition(body_num) {
        let wpos = new THREE.Vector3();
        if ((body_num == 0 || body_num == null) && this.body_a != null) {
            wpos.copy(this.wp_a);
            wpos.add(this.body_a.getPosition());
            return wpos;
        }
        if ((body_num == 1) && this.body_b != null) {
            wpos.copy(this.wp_b);
            wpos.add(this.body_b.getPosition());
            return wpos;
        }

        return wpos;
    }
}

//------------------------------------------------------------------------------
//  ボールジョイント
class BallJoint extends Joint3D {
    preCalc(delta_t) {
        super.preCalc(delta_t);

        //  ヤコビアン[3*12]
        this.J = [
            [1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0]
        ];
        //  回転拘束部分
        let ra = MB.tilde([[this.lp_a.x], [this.lp_a.y], [this.lp_a.z]]);
        if (this.body_a != null && !this.body_a.isWorldBody()) {
            let lw_a = MB.QuatToMtx(this.body_a.getQuaternion().toArray());
            let Ja = math.multiply(math.multiply(lw_a, ra), -1);
            MB.copyArray(this.J, 3, 0, Ja, 3, 3);
        }

        let rb = MB.tilde([[this.lp_b.x], [this.lp_b.y], [this.lp_b.z]]);
        if (this.body_b != null && !this.body_b.isWorldBody()) {
            let lw_b = MB.QuatToMtx(this.body_b.getQuaternion().toArray());
            let Jb = math.multiply(lw_b, rb);
            MB.copyArray(this.J, 9, 0, Jb, 3, 3);
        }

        this.JT = math.transpose(this.J);

        // Amtxの作成
        this.Amtx = math.multiply(math.multiply(this.J, this.invM), this.JT);

        //  ジョイントの位置のずれを求める
        let wpos_a = this.getWorldPosition(0);
        let wpos_b = this.getWorldPosition(1);

        this.p_err.copy(wpos_b);
        this.p_err.sub(wpos_a);
        //  エラー補正係数を掛ける
        this.p_err.multiplyScalar(0.2 / delta_t);

        //  ジョイント位置の速度の補正
        let wvel_a = new THREE.Vector3();
        let wvel_b = new THREE.Vector3();
        if (this.body_a != null) {
            wvel_a.copy(this.body_a.getLocalPointVelocity(this.lp_a));
        }
        if (this.body_b != null) {
            wvel_b.copy(this.body_b.getLocalPointVelocity(this.lp_b));
        }
        this.v_err.copy(wvel_b);
        this.v_err.sub(wvel_a);
        this.v_err.multiplyScalar(0.8);          //  ここの係数を入れないと安定しない
    }
}

//------------------------------------------------------------------------------
//  回転ジョイント
class RevoluteJoint extends Joint3D {

    constructor(body_a, lp_a, laxis_a, body_b, lp_b, laxis_b) {
        super(body_a, lp_a, body_b, lp_b);

        //  回転軸をZ軸として、拘束に必要な直交するX,Y軸を求めておく
        this.axis_a = new THREE.Matrix3();
        this.axis_b = new THREE.Matrix3();

        this.axis_a.set(0, 0, laxis_a.x,
                        0, 0, laxis_a.y,
                        0, 0, laxis_a.z);
        this.planeSpace(this.axis_a);

        this.axis_b.set(0, 0, laxis_b.x,
                        0, 0, laxis_b.y,
                        0, 0, laxis_b.z);
        this.planeSpace(this.axis_b);
    }

    preCalc(delta_t) {
        super.preCalc(delta_t);

        //  ヤコビアン[5*12]
        this.J = [
            [1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0]
        ]
 
        //  回転拘束部分
        let ra = MB.tilde([[this.lp_a.x], [this.lp_a.y], [this.lp_a.z]]);
        if (this.body_a != null && !this.body_a.isWorldBody()) {
            let lw_a = MB.QuatToMtx(this.body_a.getQuaternion().toArray());
            let Ja = math.multiply(math.multiply(lw_a, ra), -1);
            MB.copyArray(this.J, 3, 0, Ja, 3, 3);

            //  拘束軸をワールド座標に変換
            let laxis = MB.MtxToArray(this.axis_a);
            let waxis = math.multiply(lw_a, laxis);
            //  回転軸のX,Y軸をヤコビアンに設定
            let p = [[waxis[0][0], waxis[1][0], waxis[2][0]]];
            MB.copyArray(this.J, 3, 3, p, 3, 1);
            let q = [[waxis[0][1], waxis[1][1], waxis[2][1]]];
            MB.copyArray(this.J, 3, 4, q, 3, 1);
        }

        let rb = MB.tilde([[this.lp_b.x], [this.lp_b.y], [this.lp_b.z]]);
        if (this.body_b != null && !this.body_b.isWorldBody()) {
            let lw_b = MB.QuatToMtx(this.body_b.getQuaternion().toArray());
            let Jb = math.multiply(lw_b, rb);
            MB.copyArray(this.J, 9, 0, Jb, 3, 3);

            //  拘束軸をワールド座標に変換
            let laxis = MB.MtxToArray(this.axis_b);
            let waxis = math.multiply(lw_b, laxis);
            //  回転軸のX,Y軸をヤコビアンに設定
            let p = [[-waxis[0][0], -waxis[1][0], -waxis[2][0]]];
            MB.copyArray(this.J, 9, 3, p, 3, 1);
            let q = [[-waxis[0][1], -waxis[1][1], -waxis[2][1]]];
            MB.copyArray(this.J, 9, 4, q, 3, 1);
        }

        //  ヤコビアンの転置
        this.JT = math.transpose(this.J);

        // Amtxの作成
        this.Amtx = math.multiply(math.multiply(this.J, this.invM), this.JT);

        //  ジョイントの位置のずれを求める
        let wpos_a = this.getWorldPosition(0);
        let wpos_b = this.getWorldPosition(1);

        this.p_err.copy(wpos_b);
        this.p_err.sub(wpos_a);
        //  エラー補正係数を掛ける
        this.p_err.multiplyScalar(0.2 / delta_t);

    }    

    //  Z軸を元にして、直交するX,Y軸を求める
    planeSpace(mtx) {
        var n = new THREE.Vector3();
        var p = new THREE.Vector3();
        var q = new THREE.Vector3();
        n.fromArray(mtx.elements, 2 * 3);     //  Z軸

        if(n.z > Joint3D.Sqrt12)
        {
            //  p は yz 平面上
            let a = n.y*n.y + n.z*n.z;
            let k = 1.0 / Math.sqrt(a);
            p.x = 0;
            p.y = -n.z * k;
            p.z = n.y * k;
            //  q = n x p 外積
            q.x = a * k;
            q.y = -n.x * p.z;
            q.z = n.x * p.y;
        }
        else
        {
            //  p は xy 平面上
            let a = n.x*n.x + n.y*n.y;
            let k = 1.0 / Math.sqrt(a);
            p.x = -n.y * k;
            p.y = n.x * k;
            p.z = 0;
            //  q = n x p 外積
            q.x = -n.z * p.y;
            q.y = n.z * p.x;
            q.z = a * k;
        }

        //  マトリクスに設定する
        mtx.set(p.x, q.x, n.x,
                p.y, q.y, n.y,
                p.z, q.z, n.z);
    }

    //  ワールド座標系での回転軸をマトリクスで返す
    getJointAxisWorldMtx() {
        // if (this.body_a != null && !this.body_a.isWorldBody()) {

        //  拘束軸をワールド座標に変換
        let lw_a = MB.QuatToMtx(this.body_a.getQuaternion().toArray());
        let laxis = MB.MtxToArray(this.axis_a);
        let waxis = math.multiply(lw_a, laxis);

        //  マトリクスに設定して返す
        var axis_mtx = new THREE.Matrix3();

        axis_mtx.set(waxis[0][0], waxis[0][1], waxis[0][2],
                     waxis[1][0], waxis[1][1], waxis[1][2],
                     waxis[2][0], waxis[2][1], waxis[2][2]);

        return axis_mtx;
    }
}

