import * as THREE from 'three';
/**
 * three的场景
 * @type {THREE.Scene}
 * */
declare let scene: THREE.Scene;
/**
 * three的渲染器，carviz用的WebGLRenderer渲染器
 * @type {THREE.WebGLRenderer}
 * */
declare let renderer: any;
/**
 * three的相机，carviz库用的PerspectiveCamera相机
 * @type {THREE.PerspectiveCamera}
 * */
declare let camera: any;
/**
 * 绘制网格的类实例对象
 * @type { View }
 * */
declare let view: any;
/**
 * 绘制网格的类实例对象
 * @type { Controls }
 * */
declare const controls: any;
/**
 * 绘制地图实例对象
 * @type { Map }
 * */
declare let map: any;
/**
 * 绘制检查点
 * @type { Checkpoints }
 * */
declare let checkpoints: any;
/**
 * 绘制靠边停车区域
 * @type { Pullover }
 * */
declare let pullover: any;
/**
 * 绘制地图实例对象
 * @type { Adc }
 * */
declare let adc: any;
/**
 * 绘制感知障碍物
 * @type { Obstacles }
 * */
declare let obstacles: any;
/**
 * 绘制点云
 * @type { PointCloud }
 * */
declare let pointCloud: any;
/**
 * 重新渲染
 */
declare const render: () => void;
/**
 * 通过 name 删除场景中的某个对象
 * @param {string} name object3D name属性值
 */
declare const removeObjetByName: (name: any) => void;
/**
 * 初始化carviz，如果要使用carviz必须初始化
 * @param {string} id
 */
declare const init: (id: any) => void;
/**
 * 添加对象到场景中
 * @param {THREE.Object3D} object3D
 */
declare const addObjectToScene: (object: any) => void;
export {
    camera,
    renderer,
    scene,
    view,
    addObjectToScene,
    init,
    removeObjetByName,
    render,
    controls,
    map,
    adc,
    checkpoints,
    pullover,
    obstacles,
    pointCloud,
};
