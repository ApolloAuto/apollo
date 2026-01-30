import React from 'react';
import { createRoot, Root } from 'react-dom/client';
import FloatingLayer from '@dreamview/dreamview-carviz/src/EventBus/eventListeners/FloatingLayer';
import type { CSS2DRenderer } from 'three/examples/jsm/renderers/CSS2DRenderer';
import * as THREE from 'three';
import { FunctionalClass } from './FunctionalClass';
import { eventBus, MouseEventType } from '../../EventBus';
import { IThreeContext } from '../type';
import type Coordinates from '../coordinates';
import CopyMessage from '../../EventBus/eventListeners/CopyMessage';
import { disposeMesh } from '../../utils/common';

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

    computeWorldSizeForPixelSize(pixelSize: number) {
        // 摄像机距离地平面的距离
        const camera = this.context.camera;
        const distance = camera.position.distanceTo(new THREE.Vector3(0, 0, 0));
        const vFOV = THREE.MathUtils.degToRad(camera.fov); // 将视角转换为弧度
        const visibleHeight = 2 * Math.tan(vFOV / 2) * distance;
        const worldUnitPerPixel =
            visibleHeight / (this.context.renderer?.domElement?.clientHeight || window.innerHeight);
        return pixelSize * worldUnitPerPixel;
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

        // 设置 xy 平面（地面）交点
        const plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
        const planeIntersect = new THREE.Vector3();
        this.raycaster.ray.intersectPlane(plane, planeIntersect);

        // 返回地面平面的交点
        return planeIntersect;
    }

    public computeRaycasterObject(x, y) {
        const { camera, scene } = this.context;
        const { x: nx, y: ny } = this.computeNormalizationPosition(x, y);
        const raycasterTemp = new THREE.Raycaster();
        raycasterTemp.setFromCamera(new THREE.Vector2(nx, ny), camera);

        const ParkingSpaceModels = [];
        scene.children.forEach((model) => {
            if (model.name === 'ParkingSpace') {
                ParkingSpaceModels.push(model);
            }
        });

        const mesh = this.createShapeMesh();
        scene.add(mesh);

        let selectedObject;
        for (let index = 0; index < ParkingSpaceModels.length; index++) {
            const element = ParkingSpaceModels[index];
            mesh.geometry.dispose();
            const positions = element.geometry.attributes.position;
            const vertices = [];
            for (let i = 0; i < positions.count - 1; i++) {
                const index = i * 3;
                vertices.push(
                    new THREE.Vector3(positions.array[index], positions.array[index + 1], positions.array[index + 2]),
                );
            }
            const shape = new THREE.Shape(vertices);
            mesh.geometry = new THREE.ShapeGeometry(shape);
            const intersects = raycasterTemp.intersectObject(mesh);
            if (intersects.length > 0) {
                disposeMesh(mesh);
                return element;
            }
        }
        disposeMesh(mesh);
        return selectedObject;
    }

    createShapeMesh() {
        const vertices = [
            new THREE.Vector2(0, 0),
            new THREE.Vector2(0, 0),
            new THREE.Vector2(0, 0),
            new THREE.Vector2(0, 0),
        ];

        const shape = new THREE.Shape(vertices);
        const geometry = new THREE.ShapeGeometry(shape);
        const material = new THREE.MeshBasicMaterial({ color: 0xff0000, visible: false });
        const mesh = new THREE.Mesh(geometry, material);

        return mesh;
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
