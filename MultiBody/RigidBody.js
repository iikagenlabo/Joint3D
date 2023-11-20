import * as THREE from 'three';

import { MBFunc3D } from "./MBFunc3D.js";
import { DynamicsParameter } from "./DynamicsParameter.js";

//  マルチボディライブラリ
var MB = new MBFunc3D();

export class RigidBody {
    constructor() {
        //  運動状態のパラメータ
        this.dynamics = new DynamicsParameter();

        //  ルンゲ・クッタ用のワーク
        this.dyn_org = new DynamicsParameter();
        this.k = [
            new DynamicsParameter(),
            new DynamicsParameter(),
            new DynamicsParameter(),
            new DynamicsParameter()
        ];

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

    //  状態取得用
    getPosition() {
        return this.dynamics.position;
    }
    getVelocity() {
        return this.dynamics.velocity;
    }
    getAccel() {
        return this.dynamics.accel;
    }

    getQuaternion() {
        return this.dynamics.quaternion;
    }
    getOmega() {
        return this.dynamics.omega;
    }
    getDOmega() {
        return this.dynamics.d_omega;
    }

    getWorldOmega() {
        let wom = this.getOmega().clone();
        wom.applyQuaternion(this.getQuaternion());
        return wom;
    }

    isWorldBody() {
        return false;
    }

    preCalcParameter() {
        this.invMass = 1 / this.mass;
        this.invI = [
            1 / this.inertia.x,
            1 / this.inertia.y,
            1 / this.inertia.z
        ];

        this.i_mtx = [
            [this.inertia.x, 0, 0],
            [0, this.inertia.y, 0],
            [0, 0, this.inertia.z]
        ];
        this.inv_i = [
            [1 / this.inertia.x, 0, 0],
            [0, 1 / this.inertia.y, 0],
            [0, 0, 1 / this.inertia.z]
        ];
    }

    calcInertiaTensor(lw_mtx) {
        const wl_mtx = math.transpose(lw_mtx);
        return math.multiply(math.multiply(lw_mtx, this.i_mtx), wl_mtx);
    }
    //  慣性テンソルの逆数を求める
    calcInvInertiaTensor(lw_mtx) {
        //  慣性モーメントの逆数
        const inv_i = [
            [1 / this.inertia.x, 0, 0],
            [0, 1 / this.inertia.y, 0],
            [0, 0, 1 / this.inertia.z]
        ];

        //  回転マトリクス
        const wl_mtx = math.transpose(lw_mtx);

        const invT = math.multiply(math.multiply(lw_mtx, inv_i), wl_mtx);

        return invT;
    }

    //  ループ実行前の処理
    preExec() {
        //  加速度をクリアする
        this.dynamics.accel.set(0, 0, 0);
        this.dynamics.d_omega.set(0, 0, 0);
    }

    //  表示モデルの生成
    createModel() {
        //  四角を作るようにする
    }

    //  剛体に力をかける
    applyImpulse(force, l_pos) {
        //  並進加速度
        let accel = force.clone();
        accel.multiplyScalar(this.invMass);
        this.dynamics.accel.add(accel);

        //  力の向きをローカル座標系に変換
        let wlq = this.dynamics.quaternion.clone();
        wlq.conjugate();
        force.applyQuaternion(wlq);

        //  ワールド座標系でのベクトルと外積を取ってトルクを求める
        let torque = new THREE.Vector3();
        torque.crossVectors(l_pos, force);
        torque.x *= this.invI[0];
        torque.y *= this.invI[1];
        torque.z *= this.invI[2];
        //  角加速度を更新
        this.dynamics.d_omega.add(torque);
    }

    //  剛体にトルクをかける
    applyTorque(torque) {
        //  トルクをローカル座標系に変換
        let wlq = this.getQuaternion().clone();
        wlq.conjugate();
        torque.applyQuaternion(wlq);

        //  慣性モーメントで割って角加速度を求める
        torque.x *= this.invI[0];
        torque.y *= this.invI[1];
        torque.z *= this.invI[2];
        //  角加速度を更新
        this.dynamics.d_omega.add(torque);
    }

    //  ローカルの点のワールド座標での速度を求める
    getLocalPointVelocity(l_pos) {
        var p_vel = this.dynamics.omega.clone();
        p_vel.cross(l_pos);
        p_vel.applyQuaternion(this.dynamics.quaternion);        //  ワールド座標系に変換
        p_vel.add(this.dynamics.velocity);

        return p_vel;
    }

    //  クォータニオンの掛け算
    crossQuaternion(a, b) {
        let q = new THREE.Quaternion();

        q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
        q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
        q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w

        return q;
    }

    //  コリオリ力を求める
    calcCoriolisForce(omega) {
        //  角速度の外積オペレータ
        let om = [[omega.x], [omega.y], [omega.z]];
        let tilde_omega = MB.tilde(om);

        //  角加速度の計算
        //  コリオリ力(-tw * J * w)
        let torque = math.multiply(tilde_omega, math.multiply(this.i_mtx, om));
        torque = math.multiply(torque, -1);
        // torque = math.multiply(torque, 0);

        return torque;  //  [[x], [y], [z]]
    }

    //  RKの計算前の処理
    preRKCalc(step, delta_t) {
        switch (step) {
            case 0:     //  １段目
                //  初期パラメータをバックアップ
                this.dyn_org.copy(this.dynamics);
                break;
            case 1:     //  ２段目
                this.k[0].copy(this.dynamics);              //  １段目の結果を保存
                this.dynamics.copyPosQuatVelOmega(this.dyn_org);
                this.dynamics.updateStep(delta_t / 2);      //  ２段目の初期値を設定
                break;
            case 2:     //  ３段目
                this.k[1].copy(this.dynamics);              //  ２段目の結果を保存
                this.dynamics.copyPosQuatVelOmega(this.dyn_org);
                this.dynamics.updateStep(delta_t / 2);      //  ３段目の初期値を設定
                break;
            case 3:     //  ４段目
                this.k[2].copy(this.dynamics);              //  ３段目の結果を保存
                this.dynamics.copyPosQuatVelOmega(this.dyn_org);
                this.dynamics.updateStep(delta_t);          //  ４段目の初期値を設定
                break;
        }
    }

    calcRKAns(delta_t) {
        //  ４段目の結果を保存
        this.k[3].copy(this.dynamics);

        //  速度と加速度を更新する
        var ans = new DynamicsParameter();
        ans.addVelAcl(this.k[0]);
        ans.addVelAcl(this.k[1]);
        ans.addVelAcl(this.k[1]);
        ans.addVelAcl(this.k[2]);
        ans.addVelAcl(this.k[2]);
        ans.addVelAcl(this.k[3]);
        ans.scaleVelAcl(1.0 / 6.0);

        //  元の位置をコピー
        ans.copyPosQuatVelOmega(this.dyn_org);
        //  dt分進める
        ans.updateStep(delta_t);
        //  値を更新する
        this.dynamics.copy(ans);
    }

    calcRKAns2(delta_t) {
        //  ４段目の結果を保存
        this.k[3].copy(this.dynamics);

        //  速度と加速度を更新する
        var ans = new DynamicsParameter();
        ans.addPosVel(this.k[0]);
        ans.addPosVel(this.k[1]);
        ans.addPosVel(this.k[1]);
        ans.addPosVel(this.k[2]);
        ans.addPosVel(this.k[2]);
        ans.addPosVel(this.k[3]);
        ans.scalePosVel(1.0 / 6.0);

        //  元の位置をコピー
        // ans.copyPosQuatVelOmega(this.dyn_org);
        //  dt分進める
        // ans.updateStep(delta_t);
        //  値を更新する
        this.dynamics.copy(ans);
        //  クォータニオンを正規化
        this.getQuaternion().normalize();

    }

    //  クォータニオンの正規化
    normalizeQuaternion(q) {
        let len = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (len === 0) {
            return new THREE.Quaternion(0, 0, 0, 1);
        } else {
            len = 1 / len;
            return new THREE.Quaternion(q.x * len, q.y * len, q.z * len, q.w * len);
        }
    }

    updateRigidBodyRK() {
        //  加速度
        let accel = this.dynamics.accel.clone();
        if (this.gravity) {
            accel.y -= this.mass * MB.GRAVITY;
        }
        //  コリオリ力によるトルク
        let torque = this.calcCoriolisForce(this.dynamics.omega);
        let d_omega = math.multiply(this.inv_i, torque);    //  慣性モーメントで割って角加速度にする

        //  加速度と角加速度を設定
        this.dynamics.accel.copy(accel);
        this.dynamics.d_omega.x += d_omega[0][0];
        this.dynamics.d_omega.y += d_omega[1][0];
        this.dynamics.d_omega.z += d_omega[2][0];
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
        if (this.gravity) {
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
        for (var lp1 = 0; lp1 < p.length; lp1++) {
            n[lp1] = p[lp1] + dp[lp1] * dt;
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

    //  角速度からクォータニオンの時間微分を求める(dq = 1/2q*wv)
    calcDeltaQuaternion(quat, omega) {
        let vec_qw = new THREE.Quaternion(omega.x, omega.y, omega.z, 0);
        let vec_dq = this.crossQuaternion(quat, vec_qw);
        vec_dq.x *= 0.5;
        vec_dq.y *= 0.5;
        vec_dq.z *= 0.5;
        vec_dq.w *= 0.5;

        return vec_dq;
    }

    //  実行処理
    exec(delta_t) {
        // this.execRigidBody_org2(delta_t);

        //  ルンゲクッタで計算する
        this.exec_rk_test(delta_t);
    }

    //  実行処理
    execRigidBody(delta_t) {
        //  姿勢の更新 R = dRdt + R
        let omega = [[this.getOmega().x], [this.getOmega().y], [this.getOmega().z]];
        let tilde_omega = MB.tilde(omega);
        let lw_mtx = MB.QuaternionToMtx(this.getQuaternion());
        // let dotR = math.multiply(tilde_omega, lw_mtx);
        // let Rdt = math.multiply(dotR, delta_t);
        // let Rn = math.add(Rdt, lw_mtx);
        let Rn = lw_mtx;

        //  慣性テンソルの更新 I = R*I*RT
        let In = this.calcInertiaTensor(Rn);
        let invIn = this.calcInvInertiaTensor(Rn);

        //  コリオリ力
        let Tc = math.multiply(math.multiply(tilde_omega, In), omega);
        Tc = math.multiply(Tc, -1);

        //  角加速度を更新
        // let d_omega = math.multiply(invIn, Tc);
        // this.getDOmega().set(d_omega[0][0], d_omega[1][0], d_omega[2][0]);

        //  角運動量の更新 L = L + Tdt
        let Ln = math.multiply(In, omega);      // L = Iw
        Ln = math.add(Ln, math.multiply(Tc, delta_t));

        //  角速度の更新 omega = I-1 L
        let Omn = math.multiply(invIn, Ln);

        //  角速度の差分を角加速度とする
        // this.getDOmega().set(
        //     (Omn[0][0] - this.getOmega().x) / delta_t,
        //     (Omn[1][0] - this.getOmega().y) / delta_t,
        //     (Omn[2][0] - this.getOmega().z) / delta_t
        // );

        //  角速度を剛体に設定
        this.getOmega().set(Omn[0][0], Omn[1][0], Omn[2][0]);

        //  角速度でクォータニオンを更新する
        this.updateQuaternionWorld(delta_t);
    }

    //  これはだいたい良さそうな動きになるけど加速していってしまう
    execRigidBody_org2(delta_t) {
        //  姿勢の更新 R = dRdt + R
        let omega = [[this.getOmega().x], [this.getOmega().y], [this.getOmega().z]];
        let tilde_omega = MB.tilde(omega);
        let lw_mtx = MB.QuaternionToMtx(this.getQuaternion());
        // let dotR = math.multiply(tilde_omega, lw_mtx);
        // let Rdt = math.multiply(dotR, delta_t);
        // let Rn = math.add(Rdt, lw_mtx);
        let Rn = lw_mtx;

        //  慣性テンソルの更新 I = R*I*RT
        let In = this.calcInertiaTensor(Rn);
        let invIn = this.calcInvInertiaTensor(Rn);

        //  コリオリ力
        let Tc = math.multiply(math.multiply(tilde_omega, In), omega);
        Tc = math.multiply(Tc, -1);

        //  角運動量の更新 L = L + Tdt
        let Ln = math.multiply(In, omega);      // L = Iw
        Ln = math.add(Ln, math.multiply(Tc, delta_t));

        //  角速度の更新 omega = I-1 L
        let Omn = math.multiply(invIn, Ln);

        //  角速度を剛体に設定
        this.getOmega().set(Omn[0][0], Omn[1][0], Omn[2][0]);

        //  角速度でクォータニオンを更新する
        this.updateQuaternionWorld(delta_t);
    }

    //  最初の実験
    execRigidBody_org(delta_t) {
        //  姿勢の更新 R = dRdt + R
        let omega = [[this.getOmega().x], [this.getOmega().y], [this.getOmega().z]];
        let tilde_omega = MB.tilde(omega);
        let lw_mtx = MB.QuaternionToMtx(this.getQuaternion());
        let dotR = math.multiply(tilde_omega, lw_mtx);
        let Rdt = math.multiply(dotR, delta_t);
        let Rn = math.add(Rdt, lw_mtx);

        //  慣性テンソルの更新 I = R*I*RT
        let In = this.calcInertiaTensor(Rn);
        let invIn = this.calcInvInertiaTensor(Rn);

        //  コリオリ力
        let Tc = math.multiply(math.multiply(tilde_omega, In), omega);
        Tc = math.multiply(Tc, -1);

        //  角運動量の更新 L = L + Tdt
        let Ln = math.multiply(In, omega);      // L = Iw
        Ln = math.add(Ln, math.multiply(Tc, delta_t));

        //  角速度の更新 omega = I-1 L
        let Omn = math.multiply(invIn, Ln);

        //  角速度を剛体に設定
        this.getOmega().set(Omn[0][0], Omn[1][0], Omn[2][0]);

        //  角速度でクォータニオンを更新する
        this.updateQuaternionWorld(delta_t);
    }

    //  角速度からクォータニオンを更新
    updateQuaternion(delta_t) {
        //  角速度からクォータニオンの時間微分を求める(dq = 1/2q*wv)
        let vec_qw = new THREE.Quaternion(this.dynamics.omega.x, this.dynamics.omega.y, this.dynamics.omega.z, 0);
        let vec_dq = this.dynamics.quaternion.clone();
        // vec_dq.multiply(vec_qw);
        vec_dq = this.crossQuaternion(vec_dq, vec_qw);
        vec_dq.x *= 0.5 * delta_t;
        vec_dq.y *= 0.5 * delta_t;
        vec_dq.z *= 0.5 * delta_t;
        vec_dq.w *= 0.5 * delta_t;

        //  クォータニオンの更新
        this.dynamics.quaternion.x += vec_dq.x;
        this.dynamics.quaternion.y += vec_dq.y;
        this.dynamics.quaternion.z += vec_dq.z;
        this.dynamics.quaternion.w += vec_dq.w;
        this.dynamics.quaternion.normalize();
    }
    updateQuaternionWorld(delta_t) {
        //  角速度からクォータニオンの時間微分を求める(dq = 1/2wv*q)
        let vec_qw = new THREE.Quaternion(this.dynamics.omega.x, this.dynamics.omega.y, this.dynamics.omega.z, 0);
        let vec_dq = this.dynamics.quaternion.clone();
        // vec_dq.multiply(vec_qw);
        vec_dq = this.crossQuaternion(vec_qw, vec_dq);
        vec_dq.x *= 0.5 * delta_t;
        vec_dq.y *= 0.5 * delta_t;
        vec_dq.z *= 0.5 * delta_t;
        vec_dq.w *= 0.5 * delta_t;

        //  クォータニオンの更新
        this.dynamics.quaternion.x += vec_dq.x;
        this.dynamics.quaternion.y += vec_dq.y;
        this.dynamics.quaternion.z += vec_dq.z;
        this.dynamics.quaternion.w += vec_dq.w;
        this.dynamics.quaternion.normalize();
    }

    exec_org(delta_t) {
        //  通常計算
        // this.exec_normal(delta_t);
        // this.exec_rk_test(delta_t);

        //  結果を反映
        this.calcRKAns(delta_t);
    }

    //  外部から回すルンゲクッタ
    execRK(step, delta_t) {
        this.preRKCalc(step, delta_t);      //  計算用ワークに値を設定
        this.preExec();                     //  加速度をクリア
        // this.updateRigidBodyRK();
        this.execRigidBody(delta_t);
        this.k[step].copy(this.dynamics);   //  結果を保存
    }

    exec_rk_flat(delta_t) {
        this.preRKCalc(0, delta_t);
        this.preExec();
        this.updateRigidBodyRK();
        this.k[0].copy(this.dynamics);

        this.dynamics.copyPosQuatVelOmega(this.dyn_org);
        this.dynamics.updateStep(delta_t / 2);
        this.preExec();
        this.updateRigidBodyRK();
        this.k[1].copy(this.dynamics);

        this.dynamics.copyPosQuatVelOmega(this.dyn_org);
        this.dynamics.updateStep(delta_t / 2);
        this.preExec();
        this.updateRigidBodyRK();
        this.k[2].copy(this.dynamics);

        this.dynamics.copyPosQuatVelOmega(this.dyn_org);
        this.dynamics.updateStep(delta_t);
        this.preExec();
        this.updateRigidBodyRK();
        this.k[3].copy(this.dynamics);

        this.calcRKAns(delta_t);
    }

    //  外部から回すルンゲクッタのテスト
    exec_rk_test(delta_t) {
        //  ４段目まで計算
        for (var i = 0; i < 4; i++) {
            this.preRKCalc(i, delta_t);
            this.preExec();
            // this.updateRigidBodyRK();
            this.execRigidBody(delta_t);
        }
        //  結果を反映
        // this.calcRKAns(delta_t);
        this.calcRKAns2(delta_t);
    }

    //  シンプレクティック法で積分
    //  でも加速していってしまうかも
    exec_sp(delta_t) {
        //  加速度を求める
        let accel = this.accel.clone();
        if (this.gravity) {
            accel.y -= this.mass * MB.GRAVITY;
        }
        //  コリオリ力によるトルク
        let torque = this.calcCoriolisForce(this.omega);
        let d_omega = math.multiply(this.inv_i, torque);    //  慣性モーメントで割って角加速度にする
        d_omega[0][0] += this.d_omega.x;
        d_omega[1][0] += this.d_omega.y;
        d_omega[2][0] += this.d_omega.z;

        //  位置の更新 v*dt + a/2*dt^2
        let dv = this.velocity.clone();
        dv.multiplyScalar(delta_t);
        accel.multiplyScalar(0.5 * delta_t * delta_t);
        this.position.add(dv);
        this.position.add(accel);

        //  角速度からクォータニオンの時間微分を求める(dq = 1/2q*wv)
        let vec_dq = this.calcDeltaQuaternion(this.quaternion, this.omega);

        //  回転角の更新 v*dt + a/2*dt^2
        let dq = vec_dq.clone();
        dq.x *= delta_t;
        dq.y *= delta_t;
        dq.z *= delta_t;
        dq.w *= delta_t;

        let dom = new THREE.Vector3(
            d_omega[0][0] * 0.5 * delta_t * delta_t,
            d_omega[1][0] * 0.5 * delta_t * delta_t,
            d_omega[2][0] * 0.5 * delta_t * delta_t
        );
        let vec_dom = this.calcDeltaQuaternion(this.quaternion, dom);
        this.quaternion.x += dq.x + vec_dom.x;
        this.quaternion.y += dq.y + vec_dom.y;
        this.quaternion.z += dq.z + vec_dom.z;
        this.quaternion.w += dq.w + vec_dom.w;
        this.quaternion.normalize();

        //  速度を更新 v + (a + next_a)/2 * dt
        accel.multiplyScalar(delta_t);
        this.velocity.add(accel);

        //  角速度を更新
        this.omega.x += d_omega[0][0] * delta_t;
        this.omega.y += d_omega[1][0] * delta_t;
        this.omega.z += d_omega[2][0] * delta_t;
    }

    //  ルンゲクッタで更新する
    exec_rk(delta_t) {
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
        //  位置を更新
        var dvel = this.dynamics.velocity.clone();
        dvel.multiplyScalar(delta_t);
        this.dynamics.position.add(dvel);

        //  角速度の外積オペレータ
        let om = [[this.dynamics.omega.x], [this.dynamics.omega.y], [this.dynamics.omega.z]];
        let tilde_omega = MB.tilde(om);

        //  角加速度の計算
        let torque = math.multiply(tilde_omega, math.multiply(this.i_mtx, om))
        torque = math.multiply(torque, -1);
        let d_omega = math.multiply(this.inv_i, torque);

        //  角速度を更新
        d_omega = math.multiply(d_omega, delta_t);
        this.dynamics.omega.x += d_omega[0][0];
        this.dynamics.omega.y += d_omega[1][0];
        this.dynamics.omega.z += d_omega[2][0];

        //  角速度からクォータニオンの時間微分を求める(dq = 1/2q*wv)
        let vec_qw = new THREE.Quaternion(this.dynamics.omega.x, this.dynamics.omega.y, this.dynamics.omega.z, 0);
        let vec_dq = this.dynamics.quaternion.clone();
        // vec_dq.multiply(vec_qw);
        vec_dq = this.crossQuaternion(vec_dq, vec_qw);
        vec_dq.x *= 0.5 * delta_t;
        vec_dq.y *= 0.5 * delta_t;
        vec_dq.z *= 0.5 * delta_t;
        vec_dq.w *= 0.5 * delta_t;

        //  クォータニオンの更新
        this.dynamics.quaternion.x += vec_dq.x;
        this.dynamics.quaternion.y += vec_dq.y;
        this.dynamics.quaternion.z += vec_dq.z;
        this.dynamics.quaternion.w += vec_dq.w;
        this.dynamics.quaternion.normalize();
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
        let qDot = new THREE.Quaternion().setFromAxisAngle(omega.normalize(), this.omega.length() / 2).multiply(this.quaternion);
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
        let dq_mtx = this.omegaToDQuatMatrix(delta_t * 0.5);
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
        this.model.quaternion.copy(this.dynamics.quaternion);
        //  位置の設定
        this.model.position.copy(this.dynamics.position);
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
        //  (w, x, y, z)
        return [
            [0, -om.x, -om.y, -om.z],
            [om.x, 0, -om.z, om.y],
            [om.y, om.z, 0, -om.x],
            [om.z, -om.y, om.x, 0]
        ];
    }
}

//  ワールド固定オブジェクト
class WorldBody extends RigidBody
{
    constructor()
    {
        super();
    }

    isWorldBody() {
        return true;
    }
}

//  いらないところを整理しないといけないな

//  https://qiita.com/GANTZ/items/8a9d52c91cce902b44c9

