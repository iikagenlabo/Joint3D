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

    // addScene(scene) {
    //     const box = new THREE.Box3();
    //     box.setFromCenterAndSize(new THREE.Vector3(1, 1, 1), new THREE.Vector3(0, 0, 0))
    //     this.model = new THREE.Box3Helper(box, 0xffff00);
    //     scene.add(this.model);
    // }

    createModel() {
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

        //  回転角を更新する

    }

    //  位置と回転角をモデルに反映
    updatePosRot() {
        if (this.model == null) return;

        //  モデルを回転させる.
        this.model.quaternion.copy(this.quaternion);
        //  位置の設定
        this.model.position.copy(this.position);
    }

}