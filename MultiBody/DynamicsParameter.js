import * as THREE from 'three';

//  剛体の運動状態のパラメータ
class DynamicsParameter {
    constructor() {
        this.position = new THREE.Vector3();
        this.velocity = new THREE.Vector3();
        this.accel = new THREE.Vector3();

        this.quaternion = new THREE.Quaternion();
        this.omega = new THREE.Vector3();
        this.d_omega = new THREE.Vector3();
    }

    //  他のTransformの値をコピーして取り込む
    copy(trans) {
        this.position.copy(trans.position);
        this.velocity.copy(trans.velocity);
        this.accel.copy(trans.accel);

        this.quaternion.copy(trans.quaternion);
        this.omega.copy(trans.omega);
        this.d_omega.copy(trans.d_omega);
    }

    //  位置と姿勢だけコピーする
    copyPosQuatVelOmega(trans) {
        this.position.copy(trans.position);
        this.velocity.copy(trans.velocity);
        this.quaternion.copy(trans.quaternion);
        this.omega.copy(trans.omega);
    }

    copyPosQuat(trans) {
        this.position.copy(trans.position);
        this.quaternion.copy(trans.quaternion);
    }

    //  速度と加速度だけ足す
    addVelAcl(trans) {
        this.velocity.add(trans.velocity);
        this.accel.add(trans.accel);
        this.omega.add(trans.omega);
        this.d_omega.add(trans.d_omega);
    }

    //  速度と位置だけ足す
    addPosVel(trans) {
        this.velocity.add(trans.velocity);
        this.position.add(trans.position);
        this.omega.add(trans.omega);
        // this.quaternion.add(trans.quaternion);
        this.quaternion.x += trans.quaternion.x;
        this.quaternion.y += trans.quaternion.y;
        this.quaternion.z += trans.quaternion.z;
        this.quaternion.w += trans.quaternion.w;
    }

    addVel(trans) {
        this.velocity.add(trans.velocity);
        this.omega.add(trans.omega);
    }

    //  速度と加速度だけスケールを掛ける
    scaleVelAcl(scale) {
        this.velocity.multiplyScalar(scale);
        this.accel.multiplyScalar(scale);
        this.omega.multiplyScalar(scale);
        this.d_omega.multiplyScalar(scale);
    }

    scalePosVel(scale) {
        this.velocity.multiplyScalar(scale);
        this.position.multiplyScalar(scale);
        this.omega.multiplyScalar(scale);
        this.quaternion.x *= scale;
        this.quaternion.y *= scale;
        this.quaternion.z *= scale;
        this.quaternion.w *= scale;
    }

    scaleVel(scale) {
        this.velocity.multiplyScalar(scale);
        this.omega.multiplyScalar(scale);
    }

    //  クォータニオンの掛け算
    static crossQuaternion(a, b) {
        let q = new THREE.Quaternion();

        q.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        q.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
        q.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
        q.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w

        return q;
    }

    //  角速度からクォータニオンの時間微分を求める(dq = 1/2q*wv)
    static calcDeltaQuaternion(quat, omega, delta_t = 1.0) {
        let vec_qw = new THREE.Quaternion(omega.x, omega.y, omega.z, 0);
        let vec_dq = DynamicsParameter.crossQuaternion(quat, vec_qw);
        vec_dq.x *= 0.5 * delta_t;
        vec_dq.y *= 0.5 * delta_t;
        vec_dq.z *= 0.5 * delta_t;
        vec_dq.w *= 0.5 * delta_t;

        return vec_dq;
    }
    //  角速度からクォータニオンの時間微分を求める(dq = 1/2wv*q)
    static calcDeltaQuaternionWorld(quat, omega, delta_t = 1.0) {
        let vec_qw = new THREE.Quaternion(omega.x, omega.y, omega.z, 0);
        let vec_dq = DynamicsParameter.crossQuaternion(vec_qw, quat);
        vec_dq.x *= 0.5 * delta_t;
        vec_dq.y *= 0.5 * delta_t;
        vec_dq.z *= 0.5 * delta_t;
        vec_dq.w *= 0.5 * delta_t;

        return vec_dq;
    }

    //  位置と速度をdelta_t分進める
    updateStep(delta_t) {
        //  加速度で速度を更新
        var accel = this.accel.clone();
        accel.multiplyScalar(delta_t);
        this.velocity.add(accel);

        //  速度で位置を更新
        var vel = this.velocity.clone();
        vel.multiplyScalar(delta_t);
        this.position.add(vel);

        //  角加速度で角速度を更新
        var d_omega = this.d_omega.clone();
        d_omega.multiplyScalar(delta_t);
        this.omega.add(d_omega);

        //  角速度からクォータニオンを更新
        //  角速度からクォータニオンの時間微分を求める(dq = 1/2wv*q)
        let vec_dq = DynamicsParameter.calcDeltaQuaternionWorld(this.quaternion, this.omega, delta_t);

        this.quaternion.x += vec_dq.x;
        this.quaternion.y += vec_dq.y;
        this.quaternion.z += vec_dq.z;
        this.quaternion.w += vec_dq.w;
        this.quaternion.normalize();
    }

    updateStep2(delta_t) {
        //  速度で位置を更新
        var vel = this.velocity.clone();
        vel.multiplyScalar(delta_t);
        this.position.add(vel);

        //  角速度からクォータニオンを更新
        //  角速度からクォータニオンの時間微分を求める(dq = 1/2wv*q)
        let vec_dq = DynamicsParameter.calcDeltaQuaternionWorld(this.quaternion, this.omega, delta_t);

        this.quaternion.x += vec_dq.x;
        this.quaternion.y += vec_dq.y;
        this.quaternion.z += vec_dq.z;
        this.quaternion.w += vec_dq.w;
        this.quaternion.normalize();
    }
}

export { DynamicsParameter };