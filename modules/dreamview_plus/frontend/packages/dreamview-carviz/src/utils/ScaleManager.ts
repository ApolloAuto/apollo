import * as THREE from 'three';
import {PerspectiveCamera} from "three";
import {debounce, throttle} from "lodash";

class ScaleManager {
    private camera: PerspectiveCamera;
    private objects: Map<THREE.Object3D, number>;

    // 是否初始化camera
    private isCameraInitialized: boolean = false;

    constructor() {
        this.objects = new Map();
    }

    public setCamera(camera: PerspectiveCamera) {
        this.camera = camera;
        // 增加节流



        this.camera.addEventListener('change',
        throttle(() => {
            this.updateScales();
        }, 50));
        this.isCameraInitialized = true;
    }

    public add(object: THREE.Object3D, desiredSizePixels: number = 50): void {
        this.objects.set(object, desiredSizePixels);
    }

    public remove(object: THREE.Object3D): void {
        this.objects.delete(object);
    }

    private updateScales(): void {
        if (!this.isCameraInitialized) {
            return;
        }
        this.objects.forEach((desiredSizePixels, object) => {
            this.adjustScale(object, desiredSizePixels);
        });
    }

    private adjustScale(object: THREE.Object3D, desiredSizeInPixels: number): void {
        const fov = this.camera?.fov * (Math.PI / 180);
        const objectDistance = this.camera?.position.distanceTo(object.position);

        // 计算物体的包围盒
        const objectBox = new THREE.Box3().setFromObject(object);
        const objectSize = objectBox.getSize(new THREE.Vector3());

        // 取包围盒的最大尺寸作为物体的参考尺寸
        const objectHeight = Math.max(objectSize.x, objectSize.y, objectSize.z);

        // 计算当前物体的投影高度（以像素为单位）
        const currentSizeInPixels = (objectHeight / objectDistance) * (window.innerHeight / (2 * Math.tan(fov / 2)));

        // 计算需要缩放的比例
        const scale = desiredSizeInPixels / currentSizeInPixels;

        // 设置物体缩放
        object.scale.set(scale, scale, scale);
        this.camera?.updateProjectionMatrix();
    }
}

export const scaleManager = new ScaleManager();
