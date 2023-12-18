import * as THREE from 'three';
import IcAnchorMousePointer from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcAnchorMousePointer.png';
import {
    createEditingAnchorArrow,
    createEditingAnchorMarker,
} from '@dreamview/dreamview-carviz/src/components/EditingAnchorMarker';
import { IMouseDownInfo, InteractionType } from '@dreamview/dreamview-carviz/src/event/MouseInteractionEvent';
import Point from '@dreamview/dreamview-carviz/src/render/Point';
import { MouseInteractionEvent } from '@dreamview/dreamview-carviz/src/event';
import { isNil } from 'lodash';
import { FunctionalOperation, IMarker, PointData } from '../type';
import { BaseMarker, IFunctionalClassContext } from './BaseMarker';
import { eventBus, MouseEventType } from '../../EventBus';

export type CreateInitiationMarkerCallback = (operation: FunctionalOperation, position: PointData) => void;

interface IInitiationMarkerContext extends IFunctionalClassContext {}

// 起始点绘制工具
export default class InitiationMarker extends BaseMarker {
    private positions: IMarker[] = [];

    public eventHandler: MouseInteractionEvent;

    // 用于鼠标移动时创建并共享的点实例
    private currentMovePosition: IMarker = null;

    // 用于点击创建起始点的回调
    private createInitiationMarkerCallback?: CreateInitiationMarkerCallback = null;

    private marker: THREE.Mesh;

    private arrow: THREE.Mesh;

    private selectedMesh: THREE.Mesh;

    constructor(protected context: IInitiationMarkerContext) {
        super(context);
        this.name = 'InitiationMarker';
        createEditingAnchorMarker(1).then((mesh) => {
            this.marker = mesh;
        });
        createEditingAnchorArrow(0.74 * 3, 1.13 * 3).then((mesh) => {
            this.arrow = mesh;
        });
        // this.active();
    }

    public active(callback?: CreateInitiationMarkerCallback) {
        super.active();
        const { renderer } = this.context;
        this.eventHandler = new MouseInteractionEvent(
            renderer.domElement,
            {
                handleMouseDown: this.handleMouseDown,
                handleMouseMove: this.handleMouseMove,
                handleMouseMoveNotDragging: this.handleMouseMoveNotDragging,
                handleMouseUp: this.handleMouseUp,
                handleMouseLeave: this.hiddenCurrentMovePosition,
            },
            this,
        );
        renderer.domElement.style.cursor = `url('${IcAnchorMousePointer}'), default`;

        if (callback) {
            this.createInitiationMarkerCallback = callback;
        }

        return this;
    }

    public deactive() {
        super.deactive();
        const { renderer } = this.context;
        renderer.domElement.style.cursor = 'default';
        this.hiddenCurrentMovePosition();
        this.eventHandler?.destroy();

        this.createInitiationMarkerCallback = null;
        return this;
    }

    public reset() {
        this.positions.forEach((position) => {
            position.instance.remove();
        });
        this.positions = [];
        this.triggerCallback('reset');
        this.selectParkingSpace(false);
        return this;
    }

    public init(point: PointData) {
        if (isNil(point)) {
            console.error('InitiationMarker', 'point is null');
            return this;
        }

        const coordinateWithoutOffset = new THREE.Vector3(point.x, point.y, point?.z ?? 0);
        const coordinate = this.context.coordinates.applyOffset(coordinateWithoutOffset, false) as THREE.Vector3;

        this.positions.push({
            coordinate,
            direction: point.heading,
            instance: null,
        });

        // 清空之前的点
        this.positions.forEach((position) => {
            if (position.instance) {
                position.instance.remove();
            }
        });

        this.render();
        this.triggerCallback('init');
        return this;
    }

    private handleMouseDown = async (event: MouseEvent) => {
        this.currentMovePosition = null;
        const intersect = this.computeRaycasterIntersects(event.clientX, event.clientY);
        if (intersect) {
            this.currentMovePosition = {
                coordinate: intersect,
                instance: new Point(intersect, 0, {
                    ...this.context,
                    marker: this.marker.clone(),
                    arrow: this.arrow.clone(),
                })
                    .setArrowVisible(true)
                    .render(),
            };
        }
        this.selectParkingSpace(true, event.clientX, event.clientY);
    };

    private handleMouseMove = async (event: MouseEvent, mouseDownInfo: IMouseDownInfo) => {
        event.preventDefault();
        if (isNil(this.currentMovePosition)) return;

        const deltaX = event.clientX - mouseDownInfo.x;
        const deltaY = mouseDownInfo.y - event.clientY;

        const angle = Math.atan2(deltaY, deltaX);

        this.handleMouseMoveDragging(mouseDownInfo.event, angle.toFixed(2));
        this.currentMovePosition?.instance.updateDirection(angle).render();
    };

    private handleMouseUp = async (event: MouseEvent, interaction: InteractionType) => {
        if (isNil(this.currentMovePosition)) return;

        this.positions.push(this.currentMovePosition);

        // positions中只保留最新的一个点，其他清除render副作用， 不清除数据用于撤销。
        this.positions.forEach((position, index) => {
            if (index !== this.positions.length - 1) {
                position.instance.remove();
            }
        });

        this.triggerCallback('edit');
    };

    render() {
        // 获取positions最后一个值，并渲染，同时清除其他marker
        const position = this.positions[this.positions.length - 1];
        if (position) {
            // 如果已经存在point,则更新当前point， 否则创建新的point
            if (position.instance) {
                // 对于起始点，只更新点方向，无法更新位置
                position.instance.updateDirection(position.direction).render();
            } else {
                position.instance = new Point(position.coordinate, position.direction, {
                    ...this.context,
                    marker: this.marker.clone(),
                    arrow: this.arrow.clone(),
                })
                    .updateDirection(position.direction ?? 0)
                    .render();
            }
        }
    }

    // 撤销新绘制点，恢复上次绘制
    undo() {
        if (this.positionsCount === 0) return null;
        const position = this.positions.pop();
        position.instance.remove();
        const lastPosition = this.positions[this.positions.length - 1];
        this.triggerCallback('undo');
        this.selectParkingSpace(false);
        if (lastPosition) {
            lastPosition?.instance.addToScene().render();

            const { coordinates } = this.context;
            const lastPositionCoordinate = coordinates.applyOffset(lastPosition.coordinate, true) as THREE.Vector3;

            return {
                x: lastPositionCoordinate?.x,
                y: lastPositionCoordinate?.y,
                heading: lastPosition?.instance?.direction || 0,
            };
        }
        return null;
    }

    // 触发回调
    triggerCallback(operation: FunctionalOperation) {
        if (typeof this.createInitiationMarkerCallback === 'function') {
            this.createInitiationMarkerCallback(operation, this.initiationMarkerPosition);
        }
    }

    selectParkingSpace(isSelecting: boolean, x?, y?) {
        if (isSelecting) {
            // first reset the crrent mesh color
            if (this.selectedMesh) {
                this.selectParkingSpace(false);
            }
            const parkspaceObject = this.computeRaycasterObject(x, y);
            if (parkspaceObject) {
                parkspaceObject.material.color.set(parkspaceObject.userData.selectedColor);
                this.selectedMesh = parkspaceObject;
            }
        } else {
            this.selectedMesh?.material?.color?.set(this.selectedMesh.userData.color);
        }
        const { renderer, camera, scene } = this.context;
        renderer.render(scene, camera);
    }

    // 返回坐标个数
    get positionsCount() {
        return this.positions.length;
    }

    // 返回重定位点的坐标
    get initiationMarkerPosition() {
        if (this.positions.length === 0) return null;
        const lastPosition = this.positions[this.positions.length - 1];

        const { coordinates } = this.context;
        const lastPositionCoordinate = coordinates.applyOffset(lastPosition?.coordinate, true) as THREE.Vector3;
        return {
            x: lastPositionCoordinate?.x,
            y: lastPositionCoordinate?.y,
            heading: lastPosition?.instance?.direction || 0,
        };
    }
}
