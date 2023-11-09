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
        let invITa, invITb;

        if (this.body_a != null && !this.body_a.isWorldBody()) {
            this.wp_a.applyQuaternion(this.body_a.getQuaternion());
            invMa = this.body_a.invMass;
            invIa = this.body_a.invI;
            invITa = this.body_a.calcInertiaTensor();
        }
        if (this.body_b != null && !this.body_b.isWorldBody()) {
            this.wp_b.applyQuaternion(this.body_b.getQuaternion());
            invMb = this.body_b.invMass;
            invIb = this.body_b.invI;
            invITb = this.body_b.calcInertiaTensor();
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

        if (invITa != null) {
            MB.copyArray(this.invM, 3, 3, invITa, 3, 3);
        }
        if (invITb != null) {
            MB.copyArray(this.invM, 9, 9, invITb, 3, 3);
        }
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

            let wom = this.body_a.getWorldOmega();
            u[3] = [wom.x];
            u[4] = [wom.y];
            u[5] = [wom.z];
            // u[3] = [this.body_a.getOmega().x];
            // u[4] = [this.body_a.getOmega().y];
            // u[5] = [this.body_a.getOmega().z];
        }
        if (this.body_b != null && !this.body_b.isWorldBody()) {
            let vel = this.body_b.getVelocity().clone();
            vel.add(err);
            u[6] = [vel.x + err.x];
            u[7] = [vel.y + err.y];
            u[8] = [vel.z + err.z];

            let wom = this.body_b.getWorldOmega();
            u[9]  = [wom.x];
            u[10] = [wom.y];
            u[11] = [wom.z];
            // u[9] = [this.body_b.getOmega().x];
            // u[10] = [this.body_b.getOmega().y];
            // u[11] = [this.body_b.getOmega().z];
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
            this.body_a.applyTorque(this.constraintTorque);
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
