import InitiationMarker from '@dreamview/dreamview-carviz/src/render/functional/InitiationMarker';
import PathwayMarker from '@dreamview/dreamview-carviz/src/render/functional/PathwayMarker';
import RulerMarker from '@dreamview/dreamview-carviz/src/render/functional/RulerMarker';
import CopyMarker from '@dreamview/dreamview-carviz/src/render/functional/CopyMarker';
import { Vector3 } from 'three';
import { debounce } from 'lodash';
import { Carviz } from './Carviz.class';

export class RoutingEditor extends Carviz {
    public initiationMarker: InitiationMarker;

    public pathwayMarker: PathwayMarker;

    public copyMarker: CopyMarker;

    public rulerMarker: RulerMarker;

    // 摄像头触发回调
    private cameraUpdateCallback?: (cameraPosition: Vector3) => void;

    initThree() {
        super.initThree();
        this.controls.mouseButtons.LEFT = null;
        this.controls.enableRotate = false;

        // 注册事件监听器来跟踪摄像机位置的变化
        this.controls.addEventListener('change', debounce(this.handleCameraChange.bind(this), 100));
    }

    initModule() {
        super.initModule();
    }

    // 当摄像机位置或视角改变时，调用该回调
    private handleCameraChange() {
        if (this.cameraUpdateCallback) {
            const cameraCenter = this.getCameraFocus();
            const cameraCenterWithOffset = this.coordinates.applyOffset(cameraCenter, true);
            this.cameraUpdateCallback(cameraCenterWithOffset);
        }
    }

    getCameraFocus(distance = 100): Vector3 {
        // 获取摄像机的前向方向
        const forward = new Vector3(0, 0, -1).applyQuaternion(this.camera.quaternion);

        // 计算并返回视角中心点
        return new Vector3().addVectors(this.camera.position, forward.multiplyScalar(distance));
    }

    setCameraUpdateCallback(callback: (cameraCenter: Vector3) => void) {
        this.cameraUpdateCallback = callback;
    }

    deactiveAll() {
        this.initiationMarker.deactive();
        this.pathwayMarker.deactive();
        this.copyMarker.deactive();
        this.rulerMarker.deactive();
        this.indoorLocalizationMarker.deactive();
    }
}
