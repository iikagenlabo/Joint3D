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

        this.model = null;

        //  円柱用（仮）
        this.radius = 1;
        this.Width = 1;
    }

    createModel() {
        //  仮の円柱モデル
        var material = new THREE.MeshPhongMaterial({
            color: 0x4040ff,
            shading: THREE.FlatShading
        });

        var base = new THREE.Object3D();

        var geom = new THREE.CylinderGeometry(this.radius, this.radius, this.Width, 8);
        // geom.rotateZ(Math.PI);       //  90度回転
        var cy0 = new THREE.Mesh(geom, material);
        // cy0.position.y = this.length/2 - this.length/(count*2) * (i*2 + 1);
        // cy0.rotateZ(Math.PI / 2);       //  90度回転
        cy0.castShadow = true;
        cy0.receiveShadow = true;
        base.add(cy0);

        this.model = base;

        return base;
    }

    exec(delta_t) {
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

        // console.log(dE[0][0], dE[1][0], dE[2][0], dE[3][0]);

        //  回転角を更新する
        let dq = new THREE.Quaternion();
        dq.set(dE[1][0], dE[2][0], dE[3][0], dE[0][0]);
        //  試しに回転
        // dq.setFromEuler(new THREE.Euler(0.01, 0, 0));

        // dq.multiply(this.quaternion);
        // dq.normalize();
        dq.x += this.quaternion.x;
        dq.y += this.quaternion.y;
        dq.z += this.quaternion.z;
        dq.w += this.quaternion.w;
        dq.normalize();

        this.quaternion.copy(dq);
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