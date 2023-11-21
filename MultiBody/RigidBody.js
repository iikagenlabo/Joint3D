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

    preRKCalc2(step, delta_t) {
        switch (step) {
            case 0:     //  １段目
                //  初期パラメータをバックアップ
                this.dyn_org.copy(this.dynamics);
                break;
            case 1:     //  ２段目
                this.k[0].copy(this.dynamics);              //  １段目の結果を保存
                this.dynamics.copyPosQuat(this.dyn_org);
                this.dynamics.updateStep(delta_t / 2);      //  ２段目の初期値を設定
                break;
            case 2:     //  ３段目
                this.k[1].copy(this.dynamics);              //  ２段目の結果を保存
                this.dynamics.copyPosQuat(this.dyn_org);
                this.dynamics.updateStep(delta_t / 2);      //  ３段目の初期値を設定
                break;
            case 3:     //  ４段目
                this.k[2].copy(this.dynamics);              //  ３段目の結果を保存
                this.dynamics.copyPosQuat(this.dyn_org);
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
        ans.addVel(this.k[0]);
        ans.addVel(this.k[1]);
        ans.addVel(this.k[1]);
        ans.addVel(this.k[2]);
        ans.addVel(this.k[2]);
        ans.addVel(this.k[3]);
        ans.scaleVel(1.0 / 6.0);

        //  元の位置をコピー
        ans.copyPosQuat(this.dyn_org);
        //  dt分進める
        ans.updateStep2(delta_t);
        //  値を更新する
        this.dynamics.copy(ans);
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
        //  通常計算
        // this.execRigidBody_org2(delta_t);
        this.execRigidBody_org3(delta_t);

        // this.execRigidBody(delta_t);                //  これもorg2と同じ動きにはなった
        // this.dynamics.updateStep(delta_t);

        //  ルンゲクッタで計算する
        // this.exec_rk_test(delta_t);         //  RKにするとちがう動きになってしまう
        // this.exec_rk_flat2(delta_t);
    }

    //  実行処理
    execRigidBody() {
        // this.execRigidBody_org3(delta_t);
    }


    execRigidBody_nml() {
        //  姿勢の更新 R = dRdt + R
        let omega = [[this.getOmega().x], [this.getOmega().y], [this.getOmega().z]];
        let tilde_omega = MB.tilde(omega);
        let lw_mtx = MB.QuaternionToMtx(this.getQuaternion());

        //  慣性テンソルの更新 I = R*I*RT
        let In = this.calcInertiaTensor(lw_mtx);
        let invIn = this.calcInvInertiaTensor(lw_mtx);

        //  コリオリ力
        let Tc = math.multiply(tilde_omega, math.multiply(In, omega));
        Tc = math.multiply(Tc, -1);

        //  角加速度を求める
        let d_omega = math.multiply(invIn, Tc);
        this.dynamics.d_omega.set(d_omega[0][0], d_omega[1][0], d_omega[2][0]);

        // this.dynamics.updateStep(delta_t);

        // this.dynamics.debugPrint();

        if(false)
        {
        //  角運動量の更新 L = L + Tdt
        // let Ln = math.multiply(In, omega);      // L = Iw
        // Ln = math.add(Ln, math.multiply(Tc, delta_t));

        // //  角速度の更新 omega = I-1 L
        // let Omn = math.multiply(invIn, Ln);

        // //  角速度を剛体に設定
        // this.getOmega().set(Omn[0][0], Omn[1][0], Omn[2][0]);

        // //  角速度でクォータニオンを更新する
        // this.dynamics.updateStep2(delta_t);
        }
    }

    //  角運動量の変化だけにしてみる
    execRigidBody_org3(delta_t) {
        //  角速度と外積マトリクス
        let omega = [[this.getOmega().x], [this.getOmega().y], [this.getOmega().z]];
        let tilde_omega = MB.tilde(omega);
        let lwm = MB.QuaternionToMtx(this.getQuaternion());
        //  元の慣性テンソル
        let It = this.calcInertiaTensor(lwm);

        //  角速度でクォータニオンを更新する
        this.updateQuaternionWorld(delta_t);

        //  回転行列の変化量 Rdt = w*R
        // let dotR = math.multiply(tilde_omega, lwm);
        // let Rdt = math.multiply(dotR, delta_t);
        //  姿勢の更新 R = dRdt + R
        // let Rn = math.add(lwm, Rdt);
        let Rn= MB.QuaternionToMtx(this.getQuaternion());

        //  慣性テンソルの更新 I = R*I*RT
        let In = this.calcInertiaTensor(Rn);
        let invIn = this.calcInvInertiaTensor(Rn);

        //  角運動量の更新 L = L + Tdt
        let Ln = math.multiply(It, omega);      // L = Iw
        // Ln = math.add(Ln, math.multiply(Tc, delta_t));

        //  角速度の更新 omega = I-1 L
        let Omn = math.multiply(invIn, Ln);

        //  角速度を剛体に設定
        this.getOmega().set(Omn[0][0], Omn[1][0], Omn[2][0]);

        //  角速度でクォータニオンを更新する
        // this.updateQuaternionWorld(delta_t);
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
        // this.updateQuaternionWorld(delta_t);
        this.dynamics.updateStep2(delta_t);
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

    exec_rk_flat2(delta_t) {
        this.preRKCalc(0, delta_t);
        this.preExec();
        this.execRigidBody_org3(delta_t);
        // this.k[0].copy(this.dynamics);

        this.preRKCalc(1, delta_t);
        // this.dynamics.copyPosQuatVelOmega(this.dyn_org);
        // this.dynamics.updateStep(delta_t / 2);
        this.preExec();
        this.execRigidBody_org3(delta_t);
        // this.k[1].copy(this.dynamics);

        this.preRKCalc(2, delta_t);
        // this.dynamics.copyPosQuatVelOmega(this.dyn_org);
        // this.dynamics.updateStep(delta_t / 2);
        this.preExec();
        this.execRigidBody_org3(delta_t);
        // this.k[2].copy(this.dynamics);

        this.preRKCalc(3, delta_t);
        // this.dynamics.copyPosQuatVelOmega(this.dyn_org);
        // this.dynamics.updateStep(delta_t);
        this.preExec();
        this.execRigidBody_org3(delta_t);
        // this.k[3].copy(this.dynamics);

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
        this.calcRKAns(delta_t);
    }

    //  位置と回転角をモデルに反映
    updatePosRot() {
        if (this.model == null) return;

        //  モデルを回転させる.
        this.model.quaternion.copy(this.dynamics.quaternion);
        //  位置の設定
        this.model.position.copy(this.dynamics.position);
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

