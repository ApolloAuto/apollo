import * as THREE from 'three';
import IcWaypointMousePointer from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcWaypointMousePointer.png';
import {
    createPathwayAnchorArrow,
    createPathwayAnchorMarker,
} from '@dreamview/dreamview-carviz/src/components/EditingAnchorMarker';
import { IMouseDownInfo, InteractionType } from '@dreamview/dreamview-carviz/src/event/MouseInteractionEvent';
import Point from '@dreamview/dreamview-carviz/src/render/Point';
import { MouseInteractionEvent } from '@dreamview/dreamview-carviz/src/event';
import { isNil, isNumber } from 'lodash';
import { IMarker, CreatePathwayMarkerCallbackRes, PointData, FunctionalOperation } from '../type';
import { BaseMarker, IFunctionalClassContext } from './BaseMarker';

interface IPathwayMarkerContext extends IFunctionalClassContext {}

export type CreatePathwayMarkerCallback = (operation: FunctionalOperation, CreatePathwayMarkerCallbackRes) => void;

// 起始点绘制工具
export default class PathwayMarker extends BaseMarker {
    private positions: IMarker[] = [];

    public eventHandler: MouseInteractionEvent;

    // 用于鼠标移动时创建并共享的点实例
    private currentMovePosition: IMarker = null;

    private createPathwayMarkerCallback?: CreatePathwayMarkerCallback = null;

    private marker: THREE.Mesh;

    private arrow: THREE.Mesh;

    private selectedMesh: THREE.Mesh;

    constructor(protected context: IPathwayMarkerContext) {
        super(context);
        this.name = 'PathwayMarker';
        createPathwayAnchorMarker(1).then((mesh) => {
            this.marker = mesh;
        });
        createPathwayAnchorArrow(0.74 * 3, 1.13 * 3).then((mesh) => {
            this.arrow = mesh;
        });
        // this.active();
    }

    public active(callback?: CreatePathwayMarkerCallback) {
        super.active();
        const { renderer } = this.context;
        this.eventHandler = new MouseInteractionEvent(
            renderer.domElement,
            {
                handleMouseDown: this.handleMouseDown,
                handleMouseMove: this.handleMouseMove,
                handleMouseUp: this.handleMouseUp,
                handleMouseMoveNotDragging: this.handleMouseMoveNotDragging,
                handleMouseLeave: this.hiddenCurrentMovePosition,
            },
            this,
        );
        renderer.domElement.style.cursor = `url('${IcWaypointMousePointer}'), default`;

        if (callback) {
            this.createPathwayMarkerCallback = callback;
        }

        return this;
    }

    public deactive() {
        super.deactive();
        const { renderer } = this.context;
        renderer.domElement.style.cursor = 'default';
        this.eventHandler?.destroy();
        this.createPathwayMarkerCallback = null;
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

    public init(points: PointData[]) {
        if (isNil(points)) {
            console.error('PathwayMarker', 'points is null');
            return this;
        }

        const { coordinates } = this.context;
        this.reset();
        this.positions = points.map((point) => {
            const coordinateWithoutOffset = new THREE.Vector3(point.x, point.y, point?.z ?? 0);
            const coordinate = coordinates.applyOffset(coordinateWithoutOffset, false) as THREE.Vector3;
            return {
                coordinate,
                instance: null,
                direction: point.heading,
            };
        });
        this.initRender();
        this.triggerCallback('init');
        return this;
    }

    private handleMouseDown = async (event: MouseEvent) => {
        this.currentMovePosition = null;
        const intersect = this.computeRaycasterIntersects(event.clientX, event.clientY);
        if (intersect) {
            this.currentMovePosition = {
                coordinate: intersect,
                instance: new Point(intersect, undefined, {
                    ...this.context,
                    marker: this.marker.clone(),
                    arrow: this.arrow.clone(),
                })
                    .setArrowVisible(false)
                    .render(),
            };
        }
    };

    private handleMouseMove = async (event: MouseEvent, mouseDownInfo: IMouseDownInfo) => {
        event.preventDefault();

        if (isNil(this.currentMovePosition)) return;

        const deltaX = event.clientX - mouseDownInfo.x;
        const deltaY = mouseDownInfo.y - event.clientY;

        const angle = Math.atan2(deltaY, deltaX);

        this.handleMouseMoveDragging(mouseDownInfo.event, angle.toFixed(2));

        this.currentMovePosition.direction = angle;

        this.currentMovePosition?.instance.setArrowVisible(true).updateDirection(angle).render();
    };

    private handleMouseUp = async (event: MouseEvent, interaction: InteractionType) => {
        if (isNil(this.currentMovePosition)) return;

        this.positions.push(this.currentMovePosition);

        this.selectParkingSpace(true, event.clientX, event.clientY);
        this.triggerCallback('edit');
    };

    initRender() {
        this.positions.forEach(async (position) => {
            if (!position.instance) {
                position.instance = new Point(position.coordinate, position.direction, {
                    ...this.context,
                    marker: this.marker.clone(),
                    arrow: this.arrow.clone(),
                });
                if (position.direction) {
                    position.instance.setArrowVisible(true).updateDirection(position.direction);
                }
                position.instance.render();
            }
        });
    }

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
                }).render();
            }
        }
    }

    // 撤销新绘制点，恢复上次绘制
    undo() {
        if (this.positionsCount === 0) return null;
        const position = this.positions.pop();
        position.instance.remove();
        this.triggerCallback('undo');
        this.selectParkingSpace(false);
        return this.lastPosition;
    }

    // 触发回调
    triggerCallback(operation: FunctionalOperation) {
        if (typeof this.createPathwayMarkerCallback === 'function') {
            this.createPathwayMarkerCallback(operation, {
                positions: this.pathWatMarkerPosition,
                lastPosition: this.lastPosition,
            });
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
        } else if (this.selectedMesh) {
            this.selectedMesh.material.color.set(this.selectedMesh.userData.color);
        }
        const { renderer, camera, scene } = this.context;
        renderer.render(scene, camera);
    }

    // 返回最后一个点的坐标
    get lastPosition() {
        const { coordinates } = this.context;
        if (this.positions.length === 0) return null;
        const lastPosition = this.positions[this.positions.length - 1];
        const lastPositionCoordinate = coordinates.applyOffset(lastPosition?.coordinate, true) as THREE.Vector3;
        const hasHeading = lastPosition?.instance?.arrowVisible;
        return {
            x: lastPositionCoordinate?.x,
            y: lastPositionCoordinate?.y,
            ...(hasHeading ? { heading: lastPosition?.instance.direction } : {}),
        };
    }

    // 返回坐标个数
    get positionsCount() {
        return this.positions.length;
    }

    // 返回所有点的坐标
    get pathWatMarkerPosition() {
        const { coordinates } = this.context;
        return this.positions
            .filter((position) => isNumber(position?.coordinate?.x))
            .map((position) => {
                const positionCoordinate = coordinates.applyOffset(position?.coordinate, true) as THREE.Vector3;

                const hasHeading = position?.instance?.arrowVisible;
                return {
                    x: positionCoordinate.x,
                    y: positionCoordinate.y,
                    ...(hasHeading ? { heading: position.instance.direction } : {}),
                };
            });
    }
}
