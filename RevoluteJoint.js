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
        // this.J = [
        //     [1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0],
        //     [0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0],
        //     [0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0],
        //     [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0],
        //     [0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0]
        // ];

        //  まずはボールジョイントと同じにして動くのを確認してみる
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

            //  拘束軸をワールド座標に変換
            // let laxis = MB.MtxToArray(this.axis_a);
            // let waxis = math.multiply(lw_a, laxis);
            // //  回転軸のX,Y軸をヤコビアンに設定
            // let p = [[waxis[0][0], waxis[1][0], waxis[2][0]]];
            // MB.copyArray(this.J, 3, 3, p, 3, 1);
            // let q = [[waxis[0][1], waxis[1][1], waxis[2][1]]];
            // MB.copyArray(this.J, 3, 4, q, 3, 1);
        }

        let rb = MB.tilde([[this.lp_b.x], [this.lp_b.y], [this.lp_b.z]]);
        if (this.body_b != null && !this.body_b.isWorldBody()) {
            let lw_b = MB.QuatToMtx(this.body_b.getQuaternion().toArray());
            let Jb = math.multiply(lw_b, rb);
            MB.copyArray(this.J, 9, 0, Jb, 3, 3);

            //  拘束軸をワールド座標に変換
            // let laxis = MB.MtxToArray(this.axis_b);
            // let waxis = math.multiply(lw_b, laxis);
            // //  回転軸のX,Y軸をヤコビアンに設定
            // let p = [[-waxis[0][0], -waxis[1][0], -waxis[2][0]]];
            // MB.copyArray(this.J, 9, 3, p, 3, 1);
            // let q = [[-waxis[0][1], -waxis[1][1], -waxis[2][1]]];
            // MB.copyArray(this.J, 9, 4, q, 3, 1);
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
