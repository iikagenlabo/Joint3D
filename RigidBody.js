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
        this.inertia.x = 0.13;
        this.inertia.y = 0.056;
        this.inertia.z = 0.172;

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

    exec(delta_t) {
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
        vec_dq.multiply(vec_qw);
        vec_dq.x *= 0.5 * delta_t;
        vec_dq.y *= 0.5 * delta_t;
        vec_dq.z *= 0.5 * delta_t;
        vec_dq.w *= 0.5 * delta_t;

        // console.log(dE[0][0], dE[1][0], dE[2][0], dE[3][0]);

        //  回転角を更新
        let q = this.quaternion.clone();
        // q.x += vec_dq.x;
        // q.y += vec_dq.y;
        // q.z += vec_dq.z;
        // q.w += vec_dq.w;
        q.x += dE[1][0];
        q.y += dE[2][0];
        q.z += dE[3][0];
        q.w += dE[0][0];
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
            [this.quaternion.w],
            [this.quaternion.x],
            [this.quaternion.y],
            [this.quaternion.z]
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