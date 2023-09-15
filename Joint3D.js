class Joint3D
{
    static Gravity = 9.81;

    constructor(body_a, lp_a, body_b, lp_b)
    {
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

        if(this.body_a != null)
        {
            this.wp_a.applyQuaternion(this.body_a.quaternion);
            invMa = this.body_a.invMass;
            invIa = this.body_a.invI;
        }
        if(this.body_b != null)
        {
            this.wp_b.applyQuaternion(this.body_b.quaternion);
            invMb = this.body_b.invMass;
            invIb = this.body_b.invI;
        }

        //  質量マトリクスの逆数[12*12]
        this.invM = [
            [ invMa, 0, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0 ],
            [ 0, invMa, 0, 0, 0, 0,    0, 0, 0, 0, 0, 0 ],
            [ 0, 0, invMa, 0, 0, 0,    0, 0, 0, 0, 0, 0 ],
            [ 0, 0, 0, invIa[0], 0, 0, 0, 0, 0, 0, 0, 0 ],
            [ 0, 0, 0, 0, invIa[1], 0, 0, 0, 0, 0, 0, 0 ],
            [ 0, 0, 0, 0, 0, invIa[2], 0, 0, 0, 0, 0, 0 ],
            [ 0, 0, 0, 0, 0, 0,        invMb, 0, 0, 0, 0, 0 ],
            [ 0, 0, 0, 0, 0, 0,        0, invMb, 0, 0, 0, 0 ],
            [ 0, 0, 0, 0, 0, 0,        0, 0, invMb, 0, 0, 0 ],
            [ 0, 0, 0, 0, 0, 0,        0, 0, 0, invIb[0], 0, 0 ],
            [ 0, 0, 0, 0, 0, 0,        0, 0, 0, 0, invIb[1], 0 ],
            [ 0, 0, 0, 0, 0, 0,        0, 0, 0, 0, 0, invIb[2] ]
        ];
    }

    //  拘束力の計算
    //  外力を加えて方程式の右辺を作って、連立方程式を解いて拘束力を求める
    calcConstraint(delta_t) {
		//  Amtx = [J][M-1][JT] は Joint3D で作る
		let Amtx = this.Amtx;

        //  エラー補正用の差分
        let err = this.p_err.clone();

        //  速度
        let u = [[0], [0], [0], [0], [0], [0],
                 [0], [0], [0], [0], [0], [0]];
        if(this.body_a != null)
        {
            let vel = this.body_a.velocity.clone();
            vel.sub(err);
            u[0] = [vel.x];
            u[1] = [vel.y];
            u[2] = [vel.z];
            u[3] = [this.body_a.omega.x];
            u[4] = [this.body_a.omega.y];
            u[5] = [this.body_a.omega.z];
        }
        if(this.body_b != null)
        {
            let vel = this.body_b.velocity.clone();
            vel.add(err);
            u[0] = [vel.x];
            u[1] = [vel.y];
            u[2] = [vel.z];
            u[3] = [this.body_b.omega.x];
            u[4] = [this.body_b.omega.y];
            u[5] = [this.body_b.omega.z];
        }

        //  外力
        let F = [[0], [0], [0], [0], [0], [0],
                 [0], [0], [0], [0], [0], [0]];
        if(this.body_a != null)
        {
            F[1] = [-this.body_a.mass * Joint3D.Gravity];   //  重力
            //  コリオリ力
            let torque = this.body_a.calcCoriolisForce(this.body_a.omega);
            F[3] = torque[0];
            F[4] = torque[1];
            F[5] = torque[2];
        }
        if(this.body_b != null)
        {
            F[8] = [-this.body_b.mass * Joint3D.Gravity];   //  重力
            //  コリオリ力
            let torque = this.body_b.calcCoriolisForce(this.body_b.omega);
            F[9] = torque[0];
            F[10] = torque[1];
            F[11] = torque[2];
        }

		//  Bvec = J(u + [M-1][F]*dt) [3*1]
        let Bvec = math.multiply(this.invM, F);
		// Bvec = math.dotMultiply(Bvec, delta_t);
		Bvec = math.add(Bvec, u);
		Bvec = math.multiply(this.J, Bvec);

		let impulse = [0, 0, 0];

		//  連立方程式を解く A[3*3]I[3*1] = B[3*1]
		GaussSeidel(Amtx, Bvec, impulse);

        this.constraintForce.set(impulse[0], impulse[1], impulse[2]);

        return impulse;
    }

    //  剛体に拘束力を掛ける
    applyConstraintForce()
    {
        if(this.body_a != null)
        {
            let cf = this.constraintForce.clone();
            cf.multiplyScalar(-1);
            this.body_a.applyImpulse(cf, this.wp_a);
        }
        if(this.body_b != null)
        {
            this.body_b.applyImpulse(this.constraintForce, this.wp_b);
        }
    }

    //  ジョイント位置のワールド座標
    getWorldPosition(body_num)
    {
        let wpos = new THREE.Vector3();
        if((body_num == 0 || body_num == null) && this.body_a != null)
        {
            wpos.set(this.wp_a);
            wpos.add(this.body_a.position);
            return wpos;
        }
        if((body_num == 1) && this.body_b != null)
        {
            wpos.set(this.wp_b);
            wpos.add(this.body_b.position);
            return wpos;
        }

        return wpos;
    }
}


//  回転ジョイント
class RevoluteJoint extends Joint3D
{
    preCalc(delta_t) {
        super.preCalc(delta_t);

        //  ヤコビアン[3*12]
        this.J = [
            [ 1, 0, 0, 0, 0, 0,  -1, 0, 0, 0, 0, 0 ],
            [ 0, 1, 0, 0, 0, 0,  0, -1, 0, 0, 0, 0 ],
            [ 0, 0, 1, 0, 0, 0,  0, 0, -1, 0, 0, 0 ]
        ];
        //  回転拘束部分
        let ra = MB.tilde([[this.lp_a.x], [this.lp_a.y], [this.lp_a.z]]);
        if(this.body_a != null)
        {
            let lw_a = MB.QuatToMtx(this.body_a.quaternion.toArray());
            let Ja = math.multiply(math.multiply(lw_a, ra), -1);
            MB.copyArray(this.J, 3, 0, Ja, 3, 3);
        }

        let rb = MB.tilde([[this.lp_b.x], [this.lp_b.y], [this.lp_b.z]]);
        if(this.body_b != null)
        {
            let lw_b = MB.QuatToMtx(this.body_b.quaternion.toArray());
            let Jb = math.multiply(lw_b, rb);
            MB.copyArray(this.J, 9, 0, Jb, 3, 3);
        }

		this.JT = math.transpose(this.J);

        // Amtxの作成
        this.Amtx = math.multiply(math.multiply(this.J, this.invM), this.JT);

        //  ジョイントの位置のずれを求める
        let wpos_a = this.getWorldPosition(0);
        let wpos_b = this.getWorldPosition(1);

        this.p_err.copy(wpos_a);
        this.p_err.sub(wpos_b);
        //  エラー補正係数を掛ける
        this.p_err.multiplyScalar(-0.2 / delta_t);

        //  ジョイント位置の速度の補正




        //  ヤコビアン
        // //  ここでの回転部分の拘束式はワールド座標系になる
		// this.J = [
		// 	[ 1, 0, -this.wp_a.y, -1,  0,  this.wp_b.y],
		// 	[ 0, 1,  this.wp_a.x,  0, -1, -this.wp_b.x]
		// ];

		// this.JT = math.transpose(this.J);

        // // Amtxの作成
        // this.Amtx = math.multiply(math.multiply(this.J, this.invM), this.JT);

        // //  ジョイントの位置のずれを求める
        // let wpos_a = this.getWorldPosition(0);
        // let wpos_b = this.getWorldPosition(1);
        // this.p_err.set(wpos_a);
        // this.p_err.sub(wpos_b);

        // //  エラー補正係数を掛ける
        // this.p_err.scale(-0.2 / delta_t);

        // //  ジョイント位置の速度の補正
        // let wvel_a = new Vector2();
        // let wvel_b = new Vector2();
        // if(this.body_a != null)
        // {
        //     wvel_a.set(this.body_a.getLocalPointVelocity(this.lp_a));
        // }
        // if(this.body_b != null)
        // {
        //     wvel_b.set(this.body_b.getLocalPointVelocity(this.lp_b));
        // }
        // this.v_err.set(wvel_a);
        // this.v_err.sub(wvel_b);
        // this.v_err.scale(0.3);          //  ここの係数を入れないと安定しない
    }
}
