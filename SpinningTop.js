
//  テニスラケット効果を調べる剛体
class Spinner extends RigidBody {
  constructor() {
    super();
  }

  preCalcParameter() {
    //  慣性モーメントを入れておく
    // this.inertia.x = 0.13;
    // this.inertia.y = 0.056;
    // this.inertia.z = 0.172;
    this.inertia.x = 1;
    this.inertia.y = 2;
    this.inertia.z = 4;

    super.preCalcParameter();
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
    geom.rotateZ(Math.PI / 2);       //  90度回転
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
}
//------------------------------------------------------------------------------
//  剛体振り子
class RigidPendulum extends RigidBody {
  constructor() {
    super();

    this.radius = 0.2;    //  半径
    this.length = 1.0;    //  長さ

    let Iy = this.mass * this.radius * this.radius * 0.5;
    let Ixz = (this.radius * this.radius + this.length * this.length / 3) / 4 * this.mass;
    this.inertia.x = Ixz;
    this.inertia.y = Iy;
    this.inertia.z = Ixz;

    this.gravity = true;

  }

  preCalcParameter() {

    super.preCalcParameter();
  }

  createModel(col) {
    let material = new THREE.MeshPhongMaterial({
      color: col,
      shading: THREE.FlatShading
    });

    let base = new THREE.Object3D();

    let len = 0.2;    //  円錐部分の長さ

    //  円柱
    let geom = new THREE.CylinderGeometry(this.radius, this.radius, this.length - len * 2, 8);
    let cy0 = new THREE.Mesh(geom, material);
    cy0.castShadow = true;
    // cy0.receiveShadow = true;
    base.add(cy0);

    //  上の円錐
    let geom1 = new THREE.CylinderGeometry(0, this.radius, len, 8);
    let cy1 = new THREE.Mesh(geom1, material);
    cy1.position.y = this.length / 2 - len / 2;
    cy1.castShadow = true;
    base.add(cy1);

    //  下の円錐
    let geom2 = new THREE.CylinderGeometry(this.radius, 0, len, 8);
    let cy2 = new THREE.Mesh(geom2, material);
    cy2.position.y = -this.length / 2 + len / 2;
    cy2.castShadow = true;
    base.add(cy2);

    this.model = base;
    return base;
  }
}

(function () {

  //******************************************************************************
  //  メインの実行処理.
  window.onload = function () {

    init();
    animate();

  };

  var container;
  var camera, scene, renderer;
  var spotlight;

  var top_obj;          //  コマのオブジェクト.

  //  0: スピナー
  //  1: 剛体振り子
  let mode = 1;

  //  キー入力
  var key_input = new KeyInput();

  //  ポーズフラグ
  var pause = true;

  //  定数.
  var GRAVITY = 9.81;
  //  ステップ時間
  var DeltaT = 0.001;

  //  updateのループ開始数
  var UpdateLoopCnt = 10;

  var rigid_body;
  let rod;
  let joint;

  let arrow0;

  //------------------------------------------------------------------------------
  //  初期化処理.
  function init() {

    scene = new THREE.Scene();

    //  カメラの生成.
    camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.1, 10000);
    camera.position.set(0, 4, 6);        //  カメラ位置
    scene.add(camera);
    camera.updateProjectionMatrix();

    //  ライトの追加.
    scene.add(new THREE.AmbientLight(0x404040));

    var dirLight = new THREE.DirectionalLight(0xffffff, 1);
    dirLight.name = 'Dir. Light';
    dirLight.position.set(-5, 10, 3);
    dirLight.castShadow = true;
    dirLight.shadow.camera.near = 1;
    dirLight.shadow.camera.far = 20;
    dirLight.shadow.camera.right = 10;
    dirLight.shadow.camera.left = -10;
    dirLight.shadow.camera.top = 10;
    dirLight.shadow.camera.bottom = -10;
    dirLight.shadow.mapSize.width = 1024;
    dirLight.shadow.mapSize.height = 1024;
    scene.add(dirLight);

    container = document.createElement('div');
    document.body.appendChild(container);

    //  地面のマテリアル.
    var planeGeometry = new THREE.PlaneGeometry(40, 40);
    planeGeometry.rotateX(-Math.PI / 2);
    var planeMaterial = new THREE.MeshPhongMaterial({
      color: 0xa0adaf,
      //    shininess: 150,
      //    specular: 0xffffff,
      shading: THREE.SmoothShading
    });
    //  planeMaterial = new THREE.MeshBasicMaterial({ color: 0x808080 });

    var plane = new THREE.Mesh(planeGeometry, planeMaterial);
    //  plane.position.y = -200;
    plane.receiveShadow = true;
    scene.add(plane);

    //  グリッド描画.
    var helper = new THREE.GridHelper(20, 4);
    helper.position.y = 0.01;
    helper.material.opacity = 0.5;
    helper.material.transparent = true;
    scene.add(helper);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setClearColor(0xf0f0f0);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.BasicShadowMap;
    container.appendChild(renderer.domElement);

    //  カメラ操作
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.damping = 0.2;
    controls.target.set(0, 2, 0);
    controls.addEventListener('change', render);
    controls.update();
    controls.enableKeys = false;

    //  キー入力
    key_input.initialize();


    //  コマのモデルを追加する.
    top_obj = new Top();

    // top_obj.addTopModel2();
    top_obj.initialize();

    switch (mode) {
      case 0:
        //  テニスラケット効果を調べる物体
        rigid_body = new Spinner();
        scene.add(rigid_body.createModel());
        rigid_body.preCalcParameter();
        rigid_body.position.y = 2.0;
        //  初速
        rigid_body.omega.x = 0.1;
        rigid_body.omega.y = 10;
        //  表示位置更新
        rigid_body.updatePosRot();
        break;

      case 1:
        //  剛体振り子
        rod = new RigidPendulum();
        scene.add(rod.createModel(0x4040ff));
        rod.preCalcParameter();

        //  初期位置
        rod.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 5);
        //  重心位置から見た支点位置をクォータニオンで回転させて、
        //  支点の初期位置から引くと重心の初期位置になる
        var rap = new THREE.Vector3(0, -0.5, 0);
        rap.applyQuaternion(rod.quaternion);
        rod.position.copy(rap);
        rod.position.y += 2.0;

        // rod.position.y = 2.0;
        // rod.omega.y = 1.0;
        rod.updatePosRot();

        joint = new RevoluteJoint(rod, new THREE.Vector3(0, 0.5, 0), null, new THREE.Vector3(0, 0, 0));

        initArrow();
        break;
    }



  }
  //------------------------------------------------------------------------------
  //  矢印
  function initArrow() {
    var from = new THREE.Vector3(2, 2, 2);
    var to = new THREE.Vector3(0, 0, 0);
    var direction = to.clone().sub(from);
    var length = direction.length();
    arrow0 = new THREE.ArrowHelper(direction.normalize(), from, length, 0xff0000);
    scene.add(arrow0);

    // arrow1 = new THREE.ArrowHelper(direction.normalize(), from, length, 0x0000ff);
    // scene.add(arrow1);
  }

  function setArrow() {
    // let b0 = BodyArray[0];
    // let b1 = BodyArray[1];
    let j0 = joint; //JointArray[2];

    var from = j0.wp_a.clone();
    from.add(j0.body_a.position);
    var to = j0.constraintForce.clone();
    to.multiplyScalar(-0.1);
    to.add(j0.body_a.position);

    var direction = to.clone().sub(from);
    var length = direction.length();

    arrow0.position.copy(from);
    arrow0.setLength(length);
    arrow0.setDirection(direction.normalize());
  }
  //------------------------------------------------------------------------------
  //  ループ実行部分.
  function animate() {

    requestAnimationFrame(animate);
    render();

    key_input.update();               //  キー入力

    //  ポーズ切り替え
    if (key_input.trigger & key_input.pause) {
      pause = !pause;
    }

    //	実行処理.
    if (pause == false || (key_input.trigger & key_input.step)) {
      //  カメラ移動
      //    controls.update();

      // rigid_body.omega.set(0, 0, 0);
      // if(key_input.left)
      //   rigid_body.omega.z =  20;
      // if(key_input.right)
      //   rigid_body.omega.z = -20;
      // if(key_input.up)
      //   rigid_body.omega.x = -20;
      // if(key_input.down)
      //   rigid_body.omega.x =  20;


      //  コマの運動計算.
      for (var lp1 = 0; lp1 < UpdateLoopCnt; lp1++) {
        top_obj.update(key_input);
      }

      //  剛体テスト
      for (var lp1 = 0; lp1 < UpdateLoopCnt; lp1++) {
        switch (mode) {
          case 0:
            rigid_body.exec(DeltaT);
            rigid_body.updatePosRot();
            break;

          case 1:
            //  計算実行前の処理
            rod.preExec();

            //  ジョイントの計算
            joint.preCalc(DeltaT);
            //  拘束力の計算
            joint.calcConstraint(DeltaT);
            //  拘束力を剛体にかけて速度を更新する
            joint.applyConstraintForce();

            rod.exec(DeltaT);
            rod.updatePosRot();

            setArrow();
            break;
        }
      }
    }
  };

  function render() {
    renderer.render(scene, camera);
  }

  //  マルチボディライブラリ
  var MB = new MBFunc3D();

  //******************************************************************************
  //  コマobject.
  function Top() {
    //  表示用モデル.
    this.model = null;
    this.model2 = null;

    //  定数パラメータ
    this.mass = 0.032 * Math.PI;      //  質量
    this.radius = 0.03;               //  半径
    //  重心周りの慣性行列
    var it = this.mass * this.radius * this.radius;
    this.Joa = [
      [it * 0.5, 0, 0],
      [0, it, 0],
      [0, 0, it * 0.5]
    ];

    //  慣性マトリクスの逆行列
    this.invJoa = math.inv(this.Joa);
    //  コマの座標系から見た支点の位置
    this.Rap = numeric.transpose([[0, -0.03, 0]]);

    // this.KKp = 100000;    //  支点の変位に対するバネ定数　
    // this.CCp = 1000;      //  支点の速度に対するダンピング係数
    this.KKp = 2000;    //  支点の変位に対するバネ定数　
    this.CCp = 2;       //  支点の速度に対するダンピング係数

    this.Cxz = 0;         //  角速度のダンピング係数
    this.Cyy = 0;


    //  質量マトリクス.
    this.M = [
      [this.mass, 0, 0],
      [0, this.mass, 0],
      [0, 0, this.mass]
    ];

    //  運動パラメータ
    this.position = new THREE.Vector3();      //  位置
    this.velocity = new THREE.Vector3();      //  速度
    this.theta = new THREE.Vector3();      //  角度
    this.omega = new THREE.Vector3();      //  角速度
    this.quaternion = new THREE.Quaternion(); //  回転姿勢

    //  Y, Z軸の成分を取り出すための定数
    this.Dy = [[0], [1], [0]];
    this.Dz = [[0], [0], [1]];

    //  支点位置の外積オペレータ
    this.TILDERap = MB.tilde(this.Rap);
    //  重力
    this.DyMag = numeric.mul(this.Dy, this.mass * GRAVITY);

    //  オイラーパラメータの安定化のための時定数
    this.TAU = 0.1;

    //  減衰力
    this.Cxz = 0;
    this.Cyy = 0;



    //------------------------------------------------------------------------------
    //  初期条件設定
    this.testInit = function () {
      //  支点のワールド座標
      var InitRop = numeric.transpose([0, 0, 0]);
      //  支点位置の速度
      var InitVop = numeric.transpose([0, 0, 0]);
      //  初期回転姿勢(本(w,x,y,z)と要素の並びがちがう)
      var InitEoa = new THREE.Quaternion(0,
        0,
        -Math.sin(Math.PI / 12),
        Math.cos(Math.PI / 12));
      //    var InitEoa = new THREE.Quaternion(0, 0, 0, 1);

      //  初期角速度
      //    var InitOmoa = new THREE.Vector3(0, 60, 0);
      //    var InitOmoa = new THREE.Vector3(0, 120, 0);
      var InitOmoa = new THREE.Vector3(0, 320, 0);            //  3033rpm

      //  各種定数を設定する
      //  ローカル座標での支点位置の外積オペレータ
      this.TILDERap = MB.tilde(this.Rap)
      //  重心にかかる重力
      this.DyMag = numeric.mul(this.Dy, this.mass * (-GRAVITY));

      //  重心位置の初期値
      //  重心位置から見た支点位置をクォータニオンで回転させて、
      //  支点の初期位置から引くと重心の初期位置になる
      var rap = new THREE.Vector3(0, -0.03, 0);
      rap.applyQuaternion(InitEoa).multiplyScalar(-1);

      //  角速度
      this.omega.copy(InitOmoa);
      //  回転姿勢の初期化
      this.quaternion.copy(InitEoa);
      //  位置の初期化
      this.position.copy(rap);
    };

    //------------------------------------------------------------------------------
    //  微分方程式に渡すパラメータを作る
    this.makeOdeParameter = function () {
      var param = [];

      //  クォータニオン
      param[0] = this.quaternion.w;
      param[1] = this.quaternion.x;
      param[2] = this.quaternion.y;
      param[3] = this.quaternion.z;

      //  角速度
      param[4] = this.omega.x;
      param[5] = this.omega.y;
      param[6] = this.omega.z;

      //  重心位置
      param[7] = this.position.x;
      param[8] = this.position.y;
      param[9] = this.position.z;

      //  重心速度
      param[10] = this.velocity.x;
      param[11] = this.velocity.y;
      param[12] = this.velocity.z;

      return param;
    };

    //------------------------------------------------------------------------------
    //  パラメータを元に戻す
    this.updateParameter2 = function (param) {
      //  クォータニオン
      this.quaternion.w = param[0];
      this.quaternion.x = param[1];
      this.quaternion.y = param[2];
      this.quaternion.z = param[3];

      //  クォータニオンを正規化する
      this.quaternion.normalize();

      //  角速度
      this.omega.x = param[4];
      this.omega.y = param[5];
      this.omega.z = param[6];

      //  パラメータから配列を抽出
      //  計算後の回転姿勢でマトリクスを作り直す
      var Eoa = MB.makeColumnArray(param, 0, 4);   //  オイラーパラメータ
      //  オイラーパラメータから回転行列を作る
      var Coa = MB.EtoC(Eoa);

      //  重心位置
      //  -Coa * Rap
      var Roa = math.multiply(math.multiply(Coa, -1), self.Rap);
      this.position.x = Roa[0][0];
      this.position.y = Roa[1][0];
      this.position.z = Roa[2][0];

      //  速度
      var Omoa = MB.makeColumnArray(param, 4, 3);   //  角速度
      var Voa = math.multiply(Coa, math.multiply(self.TILDERap, Omoa));
      this.velocity.x = Voa[0][0];
      this.velocity.y = Voa[1][0];
      this.velocity.z = Voa[2][0];
    };
    //------------------------------------------------------------------------------
    this.updateParameter = function (param) {
      //  クォータニオン
      this.quaternion.w = param[0];
      this.quaternion.x = param[1];
      this.quaternion.y = param[2];
      this.quaternion.z = param[3];

      //  クォータニオンを正規化する
      this.quaternion.normalize();

      //  角速度
      this.omega.x = param[4];
      this.omega.y = param[5];
      this.omega.z = param[6];

      //  重心位置
      this.position.x = param[7];
      this.position.y = param[8];
      this.position.z = param[9];

      //  重心速度
      this.velocity.x = param[10];
      this.velocity.y = param[11];
      this.velocity.z = param[12];
    };

    //------------------------------------------------------------------------------
    //  状態出力
    this.printParam = function (param) {
      console.log("Quat:%f %f %f %f", param[0], param[1], param[2], param[3]);
      console.log("Pos :%f %f %f", param[7], param[8], param[9]);
    };

    //------------------------------------------------------------------------------
    //  初期化
    this.initialize = function () {
      //  初期化
      this.testInit();

      //  テスト
      var param = this.makeOdeParameter();
      // this.printParam(param);

      param = this.updatePhysics_1045(param);
      //    this.printParam(param);

      //  モデルの位置と姿勢を更新する
      //    this.updatePosRot();
    };

    this.test_update = 0;

    //------------------------------------------------------------------------------
    //  実行処理.
    this.update = function (key_input) {
      //  実行処理
      var param = this.makeOdeParameter();

      //  ルンゲクッタで積分する
      param = MB.RKSolve(this.updatePhysics_1045, param, DeltaT);

      //  this.printParam(param);
      //  計算結果をもとに戻す
      this.updateParameter2(param);

      //  キーで回転させてみる
      //    var update = this.rotateByKey;
      //  実行処理
      //    update(key_input);

      //  モデルの位置と姿勢を更新する
      this.updatePosRot();
    };

    //------------------------------------------------------------------------------
    //  位置と姿勢をモデルに反映させる
    this.updatePosRot = function () {
      if (this.model == null) return;

      //  コマのモデルを回転させる.
      this.model.quaternion.copy(this.quaternion);

      //  位置の設定
      this.model.position.copy(this.position);
      this.model.position.multiplyScalar(100);
    };

    //  ルンゲクッタルーチンから呼ばれるのでthisを保存
    var self = this;
    //------------------------------------------------------------------------------
    //  コマの微分方程式(微分代数型運動方程式)
    this.updatePhysics_1045 = function (param) {

      //  パラメータから配列を抽出
      var Eoa = MB.makeColumnArray(param, 0, 4);   //  オイラーパラメータ
      var Omoa = MB.makeColumnArray(param, 4, 3);   //  角速度
      var Roa = MB.makeColumnArray(param, 7, 3);   //  重心位置
      var Voa = MB.makeColumnArray(param, 10, 3);  //  重心速度

      //  オイラーパラメータから回転行列を作る
      var Coa = MB.EtoC(Eoa);
      //  オイラーパラメータからS行列を作る
      var Soa = MB.EtoS(Eoa);

      //  角速度から外積オペレータを作る
      var TILDEOmoa = MB.TILDE(Omoa);

      //  -Coa * TILDERap
      var PHIOmoa = math.multiply(math.multiply(Coa, -1), self.TILDERap);

      //  外力
      //    var Foa = math.add(self.DyMag, Fop);
      var Mag = self.mass * GRAVITY;
      var Foa = [[0], [-Mag], [0]];
      //  外部トルク
      var Noa = math.zeros(3, 1);

      //  右辺の式
      var B1 = math.subtract(Noa, math.multiply(math.multiply(TILDEOmoa, self.Joa), Omoa));
      var B2 = math.multiply(Coa, math.multiply(TILDEOmoa, math.multiply(self.TILDERap, Omoa)));

      //  マトリクスを設定する
      var Amtx = MB.createArray(9, 9);
      MB.copyArray(Amtx, 0, 0, self.M, 3, 3);                 //  質量
      MB.copyArray(Amtx, 3, 3, self.Joa, 3, 3);               //  慣性モーメント
      MB.copyArray(Amtx, 0, 6, MB.createUnitArray(3, 3));     //  速度拘束
      MB.copyArray(Amtx, 6, 0, MB.createUnitArray(3, 3));
      MB.copyArray(Amtx, 3, 6, PHIOmoa, 3, 3);                //  回転速度拘束
      MB.copyArrayT(Amtx, 6, 3, PHIOmoa, 3, 3);               //  回転速度拘束

      var Bmtx = MB.createArray(1, 9);
      MB.copyArray(Bmtx, 0, 0, Foa, 3, 1);                    //  外力
      MB.copyArray(Bmtx, 0, 3, B1.valueOf(), 3, 1);
      MB.copyArray(Bmtx, 0, 6, B2.valueOf(), 3, 1);

      //  連立方程式を解く
      var X = math.lusolve(Amtx, Bmtx);

      //  オイラーパラメータの時間微分(20.48)
      var DEoa = math.multiply(math.transpose(Soa), Omoa);
      DEoa = math.map(DEoa, function (val) { return val * 0.5; });       //  全体を2で割る

      //  角速度の時間微分を方程式の解から取得
      var DOmoa = [X[3], X[4], X[5]];

      //  重心位置の時間微分
      //  -Coa * Rap
      var DRoa = math.multiply(math.multiply(Coa, -1), self.Rap);

      //  速度の時間微分
      var DVoa = math.multiply(Coa, math.multiply(self.TILDERap, Omoa));

      //  結果を配列にして返す
      var DY = [];
      DY = DY.concat(MB.makeRowArray(DEoa, 0, 4));         //  角速度
      DY = DY.concat(MB.makeRowArray(DOmoa, 0, 3));         //  角加速度
      DY = DY.concat(MB.makeRowArray(DRoa, 0, 3));          //  速度
      DY = DY.concat(MB.makeRowArray(DVoa, 0, 3));          //  加速度

      return DY;
    };

    //------------------------------------------------------------------------------
    //  角速度からクォータニオンの時間微分を求めるための行列を求める
    this.makeDQuatMatrix = function (quat) {
      var mtx4 = new THREE.Matrix4();

      var x = quat.x;
      var y = quat.y;
      var z = quat.z;
      var w = quat.w;

      mtx4.set(w, -z, y, 0,
        z, w, -x, 0,
        -y, x, w, 0,
        -x, -y, -z, 0);

      return mtx4;
    };

    //------------------------------------------------------------------------------
    //  表示モデルを追加.
    this.addTopModel = function () {
      var base = new THREE.Object3D();

      var material = new THREE.MeshPhongMaterial({
        color: 0x4040c0,
        shading: THREE.FlatShading
      });

      //  CylinderGeometry(上部半径, 下部半径, 高さ, 分割数)
      //  軸.
      var geom1 = new THREE.CylinderGeometry(0.5, 0.5, 4, 16);
      var cy0 = new THREE.Mesh(geom1, material);
      cy0.position.y = -0.5;
      cy0.castShadow = true;
      cy0.receiveShadow = true;
      base.add(cy0);

      //  板.
      var geom2 = new THREE.CylinderGeometry(3, 3, 1, 16);
      var cy1 = new THREE.Mesh(geom2, material);
      cy1.position.y = 0.0;
      cy1.castShadow = true;
      cy1.receiveShadow = true;
      base.add(cy1);

      //  先.
      var geom3 = new THREE.CylinderGeometry(0.5, 0.0, 0.5, 16);
      var cy2 = new THREE.Mesh(geom3, material);
      cy2.position.y = -2.75;
      cy2.castShadow = true;
      cy2.receiveShadow = true;
      base.add(cy2);

      //  全体をシーンに登録.
      scene.add(base);

      //  オブジェクトを保存.
      this.model = base;
    };

    //  blender出力のモデルを追加
    this.addTopModel2 = function () {
      var base = new THREE.Object3D();
      var self = this;
      var loader = new THREE.JSONLoader();

      loader.crossOrigin = "*";
      var model_data = "model/top.json";
      loader.load(model_data, function (geometry, materials) {
        var mesh = new THREE.Mesh(geometry, new THREE.MeshFaceMaterial(materials));

        mesh.position.y = -3.0;
        mesh.scale.multiplyScalar(10);            //  大きさを調整.
        mesh.castShadow = true;
        mesh.receiveShadow = true;

        //  モデルをシーンに追加
        base.add(mesh);
        scene.add(base);

        //  モデルを保存.
        self.model = base;

        //  位置と姿勢を反映させる
        self.updatePosRot();
      });
    };

  }

})();

//  とりあえず回転運動させるにはどうしたらいいか見直してみよう