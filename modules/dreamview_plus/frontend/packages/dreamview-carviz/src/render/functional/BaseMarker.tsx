import React from 'react';
import { createRoot, Root } from 'react-dom/client';
import FloatingLayer from '@dreamview/dreamview-carviz/src/EventBus/eventListeners/FloatingLayer';
import type { CSS2DRenderer } from 'three/examples/jsm/renderers/CSS2DRenderer';
import * as THREE from 'three';
import { message } from '@dreamview/dreamview-ui/src/components/Message';
import { FunctionalClass } from './FunctionalClass';
import { eventBus, MouseEventType } from '../../EventBus';
import transScreenPositionToWorld from '../../utils/transScreenPositionToWorld';
import { IThreeContext } from '../type';
import type Coordinates from '../coordinates';
import CopyMessage from '../../EventBus/eventListeners/CopyMessage';

export interface IFunctionalClassContext extends IThreeContext {
    coordinates: Coordinates;
    CSS2DRenderer: CSS2DRenderer;
}

export class BaseMarker implements FunctionalClass {
    public name: string;

    public raycaster: THREE.Raycaster = new THREE.Raycaster();

    public activeState: boolean;

    private floatLayer: HTMLElement = document.createElement('div');

    public reactRoot: Root;

    constructor(protected context: IFunctionalClassContext) {}

    active() {
        if (this.floatLayer && this.floatLayer.parentNode) {
            this.floatLayer.parentNode.removeChild(this.floatLayer);
        }

        const newLayer = document.createElement('div');
        this.activeState = true;
        this.reactRoot = createRoot(newLayer);
        newLayer.className = 'floating-layer';
        newLayer.style.width = `${window.innerWidth}px`;
        newLayer.style.height = `${window.innerHeight}px`;
        newLayer.style.position = 'absolute';
        newLayer.style.top = '0';
        newLayer.style.pointerEvents = 'none';
        document.body.appendChild(newLayer);
        this.reactRoot.render(<FloatingLayer name={this.name} />);
        this.floatLayer = newLayer;
    }

    deactive() {
        this.activeState = false;
        if (this.floatLayer && this.floatLayer.parentNode) {
            this.floatLayer.parentNode.removeChild(this.floatLayer);
        }
    }

    hiddenCurrentMovePosition() {
        eventBus.emit(MouseEventType.HIDE_CURRENT_COORDINATES);
    }

    public handleMouseMoveNotDragging = (event: MouseEvent, phase: 'Start' | 'End' = 'Start') => {
        const { renderer, camera, coordinates } = this.context;

        const worldPos = this.computeRaycasterIntersects(event.clientX, event.clientY);

        if (!worldPos || typeof worldPos.x !== 'number' || typeof worldPos.y !== 'number') {
            throw new Error('Invalid world position');
        }

        const worldPosWithOffset = coordinates.applyOffset(worldPos, true);

        if (
            !worldPosWithOffset ||
            typeof worldPosWithOffset.x !== 'number' ||
            typeof worldPosWithOffset.y !== 'number'
        ) {
            throw new Error('Invalid coordinates after applying offset');
        }
        eventBus.emit(MouseEventType.CURRENT_COORDINATES, {
            data: { x: worldPosWithOffset.x.toFixed(2), y: worldPosWithOffset.y.toFixed(2), phase },
            nativeEvent: event,
        });
    };

    public handleMouseMoveDragging = (event: MouseEvent, heading?: string, phase: 'Start' | 'End' = 'Start') => {
        const { coordinates } = this.context;

        const worldPos = this.computeRaycasterIntersects(event.clientX, event.clientY);

        if (!worldPos || typeof worldPos.x !== 'number' || typeof worldPos.y !== 'number') {
            throw new Error('Invalid world position');
        }

        const worldPosWithOffset = coordinates.applyOffset(worldPos, true);

        if (
            !worldPosWithOffset ||
            typeof worldPosWithOffset.x !== 'number' ||
            typeof worldPosWithOffset.y !== 'number'
        ) {
            throw new Error('Invalid coordinates after applying offset');
        }
        eventBus.emit(MouseEventType.CURRENT_COORDINATES, {
            data: { x: worldPosWithOffset.x.toFixed(2), y: worldPosWithOffset.y.toFixed(2), phase, heading },
            nativeEvent: event,
        });
    };

    // 定义一个函数，接受一个字符串作为参数，复制到剪贴板
    async copyMessage(msg: string) {
        try {
            await navigator.clipboard.writeText(msg);

            this.renderReactComponent(<CopyMessage success />);
        } catch (err) {
            console.error('复制失败: ', err);
            this.renderReactComponent(<CopyMessage />);
        }
    }

    public computeRaycasterIntersects(x, y) {
        const { camera, scene } = this.context;
        const { x: nx, y: ny } = this.computeNormalizationPosition(x, y);
        this.raycaster.setFromCamera(new THREE.Vector2(nx, ny), camera);

        // 计算与射线相交的对象
        const intersects = this.raycaster.intersectObjects(scene.children, true);

        // 如果与物体有交点，返回第一个交点的坐标
        if (intersects.length > 0) {
            return intersects[0].point;
        }
        // 如果没有交点，则计算与xy平面的交点
        const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
        const planeIntersect = new THREE.Vector3();
        this.raycaster.ray.intersectPlane(plane, planeIntersect);

        return planeIntersect;
    }

    public computeRaycasterObject(x, y) {
        const { camera, scene } = this.context;
        const { x: nx, y: ny } = this.computeNormalizationPosition(x, y);
        const raycasterTemp = new THREE.Raycaster();
        raycasterTemp.setFromCamera(new THREE.Vector2(nx, ny), camera);

        let ParkingSpaceModels = [];
        scene.children.forEach((model) => {
            if (model.name === "ParkingSpace") {
                ParkingSpaceModels.push(model);
            }
        });

        let selectedObject;
        for (let index = 0; index < ParkingSpaceModels.length; index++) {
            const element = ParkingSpaceModels[index];
            let box3 = new THREE.Box3().setFromObject(element);
            const point = new THREE.Vector3();
            const isIntersecting = raycasterTemp.ray.intersectBox(box3, point);
            if (isIntersecting) {
                selectedObject = element;
                break;
            }
        }
        return selectedObject;
    }

    public computeNormalizationPosition(x, y) {
        const { renderer } = this.context;
        const rect = renderer.domElement.getBoundingClientRect();
        return {
            x: ((x - rect.left) / rect.width) * 2 - 1,
            y: -((y - rect.top) / rect.height) * 2 + 1,
        };
    }

    public renderReactComponent(component: React.ReactElement, duration = 3000) {
        // 创建新的根，渲染React组件，一段时间后删除。
        const div = document.createElement('div');
        const root = createRoot(div);
        root.render(component);
        document.body.appendChild(div);
        setTimeout(() => {
            root.unmount();
            document.body.removeChild(div);
        }, duration);
    }
}
