import * as THREE from 'three';

import { RigidBody } from "./MultiBody/RigidBody.js";

//  テニスラケット効果を調べる剛体
export class Spinner extends RigidBody {
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
