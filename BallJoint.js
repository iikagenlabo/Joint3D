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
