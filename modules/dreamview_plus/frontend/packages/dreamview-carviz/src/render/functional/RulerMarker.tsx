import * as THREE from 'three';
import { type Mesh, Vector3 } from 'three';
import { MeshLine, MeshLineMaterial } from 'three.meshline';
import IcRulerMousePointer from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcRulerMousePointer.png';
import { createToolDashedLine, createToolMarker } from '@dreamview/dreamview-carviz/src/components/EditingAnchorMarker';
import { IMouseDownInfo } from '@dreamview/dreamview-carviz/src/event/MouseInteractionEvent';
import Polyline from '@dreamview/dreamview-carviz/src/render/Polyline';
import { ContinuousMouseEvent } from '@dreamview/dreamview-carviz/src/event';
import { ContinuousMouseInteractionType } from '@dreamview/dreamview-carviz/src/event/ContinuousMouseEvent';
import { createRoot } from 'react-dom/client';
import { CSS2DObject } from 'three/examples/jsm/renderers/CSS2DRenderer';
import { isNumber } from 'lodash';
import { IPolyMarker } from '../type';
import FanMarker from '../Fan';
import { BaseMarker, IFunctionalClassContext } from './BaseMarker';
import LengthLabel from '../../labels/LengthLabel';
import CloseLabel from '../../labels/CloseLabel';
import { eventBus, MouseEventType } from '../../EventBus';
import { areVerticesValid } from '../../utils/common';
import {scaleManager} from "../../utils/ScaleManager";

interface IRulerMarkerContext extends IFunctionalClassContext {}

// 起始点绘制工具
export default class RulerMarker extends BaseMarker {
    public eventHandler: ContinuousMouseEvent;

    public polylines: IPolyMarker[] = [];

    /** 鼠标拖拽时的虚线 */
    private dashedLine: Mesh;

    /** 鼠标拖拽时的扇形 */
    private fan: FanMarker;

    private roots = new Map();

    private totalLengthLabels: CSS2DObject[] = [];

    private closeLabels: CSS2DObject[] = [];

    private marker: Mesh;

    constructor(protected context: IRulerMarkerContext) {
        super(context);
        this.name = 'RulerMarker';
        // this.active();
    }

    public active() {
        super.active();
        const radius = this.computeWorldSizeForPixelSize(10);
        createToolMarker(radius).then((marker) => {
            this.marker = marker;
        });
        const { renderer } = this.context;
        this.eventHandler = new ContinuousMouseEvent(
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
        renderer.domElement.style.cursor = `url('${IcRulerMousePointer}'), default`;
    }

    public deactive() {
        super.deactive();
        const { renderer } = this.context;
        renderer.domElement.style.cursor = 'default';
        this.eventHandler?.destroy();
        this.reset();
    }

    public reset() {
        const { scene, renderer, camera, CSS2DRenderer } = this.context;
        this.polylines.forEach((polyline) => {
            polyline.instance.remove();
        });
        this.polylines = [];
        scene?.remove(this.dashedLine);
        this.dashedLine = null;
        this.fan?.remove();

        this.totalLengthLabels.forEach((label) => {
            scene.remove(label);
        });
        this.totalLengthLabels = [];

        this.closeLabels.forEach((label) => {
            scene.remove(label);
        });
        this.closeLabels = [];
        renderer.render(scene, camera);
        CSS2DRenderer.render(scene, camera);
    }

    private currentDashedVertices: Vector3[] = [];

    private updateDashedLine(vertices: Vector3[]) {
        if (vertices.length !== 2) {
            console.error('updateDashedLine expects exactly two vertices');
            return;
        }

        if (areVerticesValid(vertices) === false) {
            console.error('Invalid vertices detected:', vertices);
            return;
        }

        if (
            this.currentDashedVertices.length === 2 &&
            this.currentDashedVertices[0].equals(vertices[0]) &&
            this.currentDashedVertices[1].equals(vertices[1])
        ) {
            return;
        }

        this.currentDashedVertices = vertices.slice();

        const distance = vertices[0].distanceTo(vertices[1]);
        const lineWidth = this.computeWorldSizeForPixelSize(6);
        const color = 0x3288fa;
        const dashArray = (1 / distance) * 0.5;

        if (this.dashedLine) {
            const material = new MeshLineMaterial({
                color,
                lineWidth,
                dashArray,
            });
            this.updateMeshLine(this.dashedLine, vertices, material);
        } else {
            this.dashedLine = createToolDashedLine(vertices);
        }
    }

    private updateFan(startPoint: Vector3, midPoint: Vector3, endPoint: Vector3) {
        if (this.fan) {
            this.fan.updatePoints(startPoint, midPoint, endPoint);
        } else {
            this.fan = new FanMarker({
                ...this.context,
                fanColor: 0x1fcc4d,
                borderWidth: this.computeWorldSizeForPixelSize(6),
                borderColor: 0x1fcc4d,
                borderType: 'dashed',
            });
        }
    }

    private updateMeshLine(mesh: Mesh, vertices: Vector3[], material?: MeshLineMaterial) {
        const { scene } = this.context;

        if (areVerticesValid(vertices) === false) {
            console.error('Invalid vertices detected:', vertices);
            return;
        }

        let geometry: THREE.BufferGeometry;
        if (mesh.geometry) {
            geometry = mesh.geometry as THREE.BufferGeometry;
            const positionAttribute = geometry.getAttribute('position') as THREE.BufferAttribute;

            let verticesChanged = false;
            for (let i = 0; i < vertices.length; i += 1) {
                if (
                    positionAttribute.getX(i) !== vertices[i].x ||
                    positionAttribute.getY(i) !== vertices[i].y ||
                    positionAttribute.getZ(i) !== vertices[i].z
                ) {
                    verticesChanged = true;
                    break;
                }
            }

            if (!verticesChanged) return;

            geometry.setFromPoints(vertices);
            geometry.attributes.position.needsUpdate = true;
        } else {
            geometry = new THREE.BufferGeometry().setFromPoints(vertices);
        }

        const line = new MeshLine();
        line.setGeometry(geometry);
        mesh.geometry = line.geometry;
        mesh.material = material || mesh.material;

        if (!mesh.parent) scene.add(mesh);
    }

    private handleMouseDown = (event: MouseEvent, interaction: ContinuousMouseInteractionType) => {
        const { scene } = this.context;
        if (interaction === 'click') {
            if (this.dashedLine) scene.add(this.dashedLine);
        }
    };

    public override handleMouseMoveDragging = (event: MouseEvent, length: string, phase: 'Start' | 'End' = 'Start') => {
        eventBus.emit(MouseEventType.CURRENT_LENGTH, {
            data: { length, phase },
            nativeEvent: event,
        });
    };

    private handleMouseMove = async (event: MouseEvent, mouseDownInfo: IMouseDownInfo) => {
        event.preventDefault();
        const coordinates = this.polylines.slice(-1)[0]?.coordinates;
        const lastPosition = coordinates?.slice(-1)[0];

        if (lastPosition) {
            const endPosition = this.computeRaycasterIntersects(event.clientX, event.clientY);
            if (!endPosition) return;
            const vertices = [lastPosition, endPosition];

            const length = lastPosition.distanceTo(endPosition);
            if (isNumber(length) && length > 0) {
                this.handleMouseMoveDragging(event, length.toFixed(2), 'End');

                this.updateDashedLine(vertices);
            }
        }

        if (coordinates?.length >= 2) {
            const lastTwoPosition = coordinates.slice(-2);
            if (lastTwoPosition && lastTwoPosition.length === 2) {
                const endPosition = this.computeRaycasterIntersects(event.clientX, event.clientY);
                if (endPosition) {
                    this.updateFan(lastTwoPosition[0], lastTwoPosition[1], endPosition);
                }
            }
        }

        await this.render();
    };

    private handleMouseUp = async (event: MouseEvent, interaction: ContinuousMouseInteractionType) => {
        const { scene } = this.context;
        const planeIntersect = this.computeRaycasterIntersects(event.clientX, event.clientY);
        if (interaction === 'click') {
            if (this.polylines.length === 0) {
                this.polylines = [
                    {
                        coordinates: [],
                    },
                ];
            }
            const lastPolyline = this.polylines[this.polylines.length - 1];
            lastPolyline.coordinates.push(planeIntersect);
        } else if (interaction === 'doubleClick' || interaction === 'rightClick') {
            const lastPolyline = this.polylines[this.polylines.length - 1];
            if (interaction === 'doubleClick') {
                if (lastPolyline.coordinates.length > 2) {
                    // 多于两个点时，再删除最后一个点，否则保留
                    lastPolyline.coordinates.pop();
                    lastPolyline?.instance.updateVertices(lastPolyline.coordinates, true);
                }
            }
            this.fan?.remove();
            this.fan = null;

            let totalLength = 0;

            lastPolyline.coordinates.forEach((coordinate, index) => {
                if (index >= 1) {
                    totalLength += coordinate.distanceTo(lastPolyline.coordinates[index - 1]);
                }
            });

            this.totalLengthLabels.push(this.createOrUpdateTotalLengthLabel(totalLength));
            this.closeLabels.push(this.createOrUpdateCloseLabel(lastPolyline));

            this.renderLabel();

            // const endPosition = lastPolyline.coordinates.slice(-1)[0];
            // // 偏移量
            // const n = 4;

            // if (endPosition) {
            //     const closeMarker = await createCloseMarker(1, {
            //         name: 'closePolyline',
            //     });
            //     const position = endPosition.clone();
            //     position.x -= n;
            //     position.y -= 0.4;
            //     position.z = 0.5;
            //     if (this.isOutOfBoundary(position)) {
            //         position.x += 2 * n;
            //     }
            //     closeMarker.position.copy(position);
            //
            //     // const mouseOverEvent = new MouseOverEvent(renderer.domElement, {
            //     //     handleMouseMove: (event: MouseEvent) => {
            //     //         const { camera, scene } = this.context;
            //     //         const { x: nx, y: ny } = this.computeNormalizationPosition(event.clientX, event.clientY);
            //     //         this.raycaster.setFromCamera(new THREE.Vector2(nx, ny), camera);
            //     //
            //     //         // 计算与射线相交的对象
            //     //         const intersects = this.raycaster.intersectObjects(scene.children, true);
            //     //
            //     //         // 如果与物体有交点，返回第一个交点的坐标
            //     //         if (intersects.length > 0) {
            //     //             console.log(intersects);
            //     //         }
            //     //     },
            //     // });
            //
            //     // scene.add(closeMarker);
            // }

            scene.remove(this.dashedLine);
            this.currentDashedVertices = [];
            this.dashedLine = null;
            this.polylines.push({
                coordinates: [],
            });
        }
        await this.render();
    };

    private createOrUpdateTotalLengthLabel(length, existingElement = null) {
        const labelComponent = <LengthLabel totalLength={length} />;

        if (existingElement) {
            let root = this.roots.get(existingElement);
            if (!root) {
                root = createRoot(existingElement);
                this.roots.set(existingElement, root);
            }
            root.render(labelComponent);
            return existingElement;
        }
        const element = document.createElement('div');
        const root = createRoot(element);
        this.roots.set(element, root);
        root.render(labelComponent);
        return new CSS2DObject(element);
    }

    private clearThePolyline(polyline: IPolyMarker) {
        const { scene, camera, CSS2DRenderer } = this.context;
        const polylineIndex = this.polylines.findIndex((item) => item === polyline);
        if (polylineIndex > -1) {
            this.polylines.splice(polylineIndex, 1)[0].instance.remove();
            const closeLabel = this.closeLabels.splice(polylineIndex, 1)[0];
            const totalLengthLabel = this.totalLengthLabels.splice(polylineIndex, 1)[0];
            scene.remove(closeLabel, totalLengthLabel);
        }
        CSS2DRenderer.render(scene, camera);
    }

    private createOrUpdateCloseLabel(polyline: IPolyMarker, existingElement = null) {
        const labelComponent = (
            <CloseLabel polyline={polyline} clearThePolyline={(polyline) => this.clearThePolyline(polyline)} />
        );

        if (existingElement) {
            let root = this.roots.get(existingElement);
            if (!root) {
                root = createRoot(existingElement);
                this.roots.set(existingElement, root);
            }
            root.render(labelComponent);
            return existingElement;
        }
        const element = document.createElement('div');
        const root = createRoot(element);
        this.roots.set(element, root);
        root.render(labelComponent);
        return new CSS2DObject(element);
    }

    private computeScreenPosition(position: THREE.Vector3): { x: number; y: number } {
        const { camera, renderer } = this.context;
        const vector = position.clone().project(camera);
        vector.x = Math.round(((vector.x + 1) * renderer.domElement.offsetWidth) / 2);
        vector.y = Math.round(((-vector.y + 1) * renderer.domElement.offsetHeight) / 2);
        return vector;
    }

    async render() {
        if (this.polylines.length === 0) return;
        const lastPolyline = this.polylines[this.polylines.length - 1];
        if (lastPolyline.instance) {
            lastPolyline.instance.updateVertices(lastPolyline.coordinates).render();
        } else {
            const marker = this.marker?.clone();
            lastPolyline.instance = new Polyline({
                ...this.context,
                polylineColor: 0x3288fa,
                lineWidth: this.computeWorldSizeForPixelSize(6),
                fanColor: 0x1fcc4d,
                marker,
                label: 'length',
            })
                .updateVertices(lastPolyline.coordinates)
                .render();
        }
    }

    renderLabel() {
        const { scene, camera, CSS2DRenderer } = this.context;
        if (this.totalLengthLabels.length > 0) {
            const lastTotalLengthLabel = this.totalLengthLabels[this.totalLengthLabels.length - 1];
            const lastCloseLabel = this.closeLabels[this.closeLabels.length - 1];
            if (lastTotalLengthLabel) {
                const lastPosition = this.polylines[this.totalLengthLabels.length - 1]?.coordinates.splice(-1)[0];
                if (lastPosition) {
                    const totallengthLabelPosition = lastPosition.clone();
                    const closeLabelPosition = lastPosition.clone();
                    totallengthLabelPosition.x -= 0.4;
                    totallengthLabelPosition.y -= 1;
                    totallengthLabelPosition.z = 0;
                    lastTotalLengthLabel.position.copy(totallengthLabelPosition);
                    closeLabelPosition.x += 1.5;
                    closeLabelPosition.y -= 1.5;
                    closeLabelPosition.z = 0;
                    lastCloseLabel.position.copy(closeLabelPosition);
                    scene.add(lastTotalLengthLabel, lastCloseLabel);
                }
            }
            CSS2DRenderer.render(scene, camera);
        }
    }
}
