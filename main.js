import * as THREE from 'three';

import { OrbitControls } from 'three/addons/OrbitControls.js';
import { ConvexGeometry } from 'three/addons/ConvexGeometry.js';
import * as BufferGeometryUtils from 'three/addons/BufferGeometryUtils.js';
import { KeyInput } from './jslib/KeyInput.js';

import { Spinner } from "./RigidBodyObjects.js";

let group, camera, scene, renderer;

//  キー入力
let key_input = new KeyInput();
//  ポーズフラグ
let pause = true;
//  ステップ時間
let DeltaT = 0.01;

let rigid_body;

//  実行処理
init();
animate();

function init() {

    scene = new THREE.Scene();

    renderer = new THREE.WebGLRenderer( { antialias: true } );
    renderer.setPixelRatio( window.devicePixelRatio );
    renderer.setSize( window.innerWidth, window.innerHeight );
    document.body.appendChild( renderer.domElement );

    // camera

    camera = new THREE.PerspectiveCamera( 40, window.innerWidth / window.innerHeight, 0.1, 10000 );
    camera.position.set( 0, 4, 6 );
    scene.add( camera );
    camera.updateProjectionMatrix();

    // controls

    const controls = new OrbitControls( camera, renderer.domElement );
    controls.damping = 0.2;
    controls.target.set(0, 2, 0);
    controls.minDistance = 1;
    controls.maxDistance = 50;
    // controls.maxPolarAngle = Math.PI / 2;
    controls.update();

    // ambient light

    scene.add( new THREE.AmbientLight( 0x666666 ) );

    // point light

    const light = new THREE.PointLight( 0xffffff, 3, 0, 0 );
    camera.add( light );

    //  地面のマテリアル
    var planeGeometry = new THREE.PlaneGeometry(40, 40);
    planeGeometry.rotateX(-Math.PI / 2);
    var planeMaterial = new THREE.MeshPhongMaterial({
      color: 0xa0adaf,
      //    shininess: 150,
      //    specular: 0xffffff,
      shading: THREE.SmoothShading
    });
    //  地面
    var plane = new THREE.Mesh(planeGeometry, planeMaterial);
    plane.receiveShadow = true;
    scene.add(plane);

    //  グリッド描画
    var helper = new THREE.GridHelper(20, 4);
    helper.position.y = 0.01;
    helper.material.opacity = 0.5;
    helper.material.transparent = true;
    scene.add(helper);

    //  キー入力
    key_input.initialize();


    // helper

    scene.add( new THREE.AxesHelper( 20 ) );

    // textures

    const loader = new THREE.TextureLoader();
    const texture = loader.load( 'textures/disc.png' );
    texture.colorSpace = THREE.SRGBColorSpace;

    group = new THREE.Group();
    scene.add( group );

    // points

    let dodecahedronGeometry = new THREE.DodecahedronGeometry( 10 );

    // if normal and uv attributes are not removed, mergeVertices() can't consolidate indentical vertices with different normal/uv data

    dodecahedronGeometry.deleteAttribute( 'normal' );
    dodecahedronGeometry.deleteAttribute( 'uv' );

    dodecahedronGeometry = BufferGeometryUtils.mergeVertices( dodecahedronGeometry );

    const vertices = [];
    const positionAttribute = dodecahedronGeometry.getAttribute( 'position' );

    for ( let i = 0; i < positionAttribute.count; i ++ ) {

        const vertex = new THREE.Vector3();
        vertex.fromBufferAttribute( positionAttribute, i );
        vertices.push( vertex );

    }

    const pointsMaterial = new THREE.PointsMaterial( {
        color: 0x0080ff,
        map: texture,
        size: 1,
        alphaTest: 0.5
    } );

    const pointsGeometry = new THREE.BufferGeometry().setFromPoints( vertices );

    const points = new THREE.Points( pointsGeometry, pointsMaterial );
    group.add( points );

    // convex hull

    const meshMaterial = new THREE.MeshLambertMaterial( {
        color: 0xffffff,
        opacity: 0.5,
        side: THREE.DoubleSide,
        transparent: true
    } );

    const meshGeometry = new ConvexGeometry( vertices );

    const mesh = new THREE.Mesh( meshGeometry, meshMaterial );
    group.add( mesh );

    //
   
    window.addEventListener( 'resize', onWindowResize );

    initMode(0);
}

function initMode(mode) {

        //  テニスラケット効果を調べる物体
        rigid_body = new Spinner();
        scene.add(rigid_body.createModel());
        rigid_body.preCalcParameter();
        rigid_body.getPosition().y = 2.0;
        //  初速
        rigid_body.getOmega().x = 0.1;
        rigid_body.getOmega().y = 10;
        //  表示位置更新
        rigid_body.updatePosRot();


}

function onWindowResize() {

    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();

    renderer.setSize( window.innerWidth, window.innerHeight );

}

function animate() {

    requestAnimationFrame( animate );
    render();

    key_input.update();               //  キー入力

    //  ポーズ切り替え
    if (key_input.trigger & key_input.pause) {
      pause = !pause;
    }

    //	実行処理.
    if (pause == false || (key_input.trigger & key_input.step)) {
        group.rotation.y += 0.005;

        rigid_body.exec(DeltaT);
        rigid_body.updatePosRot();
    }
}

function render() {

    renderer.render( scene, camera );

}
