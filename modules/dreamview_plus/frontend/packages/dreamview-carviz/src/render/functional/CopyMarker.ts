import * as THREE from 'three';
import { MeshLine, MeshLineMaterial } from 'three.meshline';
import IcCopyMousePointer from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcCopyMousePointer.png';
import {
    createToolDashedLine,
    createToolMarker,
    createToolSolidLine,
} from '@dreamview/dreamview-carviz/src/components/EditingAnchorMarker';
import { IMouseDownInfo } from '@dreamview/dreamview-carviz/src/event/MouseInteractionEvent';
import Point from '@dreamview/dreamview-carviz/src/render/Point';
import { ContinuousMouseEvent } from '@dreamview/dreamview-carviz/src/event';
import { type Mesh, Vector3 } from 'three';
import { ContinuousMouseInteractionType } from '@dreamview/dreamview-carviz/src/event/ContinuousMouseEvent';
import { IMarker } from '../type';
import { BaseMarker, IFunctionalClassContext } from './BaseMarker';
import { areVerticesValid } from '../../utils/common';

interface ICopyMarkerContext extends IFunctionalClassContext {}

// 起始点绘制工具
export default class CopyMarker extends BaseMarker {
    private positions: IMarker[] = [];

    public eventHandler: ContinuousMouseEvent;

    private dashedLine: Mesh;

    private solidLine: Mesh;

    // 当前绘制状态，是否是初始化点
    private isInitiation = true;

    private marker: Mesh;

    constructor(protected context: ICopyMarkerContext) {
        super(context);
        this.name = 'CopyMarker';
        createToolMarker(0.5).then((mesh) => {
            this.marker = mesh;
        });
        // this.active();
    }

    public active() {
        super.active();
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
        renderer.domElement.style.cursor = `url('${IcCopyMousePointer}'), default`;
    }

    public deactive() {
        super.deactive();
        const { renderer } = this.context;
        renderer.domElement.style.cursor = 'default';
        this.eventHandler?.destroy();
        this.reset();
    }

    public reset() {
        const { scene } = this.context;

        this.positions.forEach((position) => {
            if (!position.instance) {
                console.error('CopyMarker', 'position.instance is null');
                return;
            }

            position.instance.remove();
        });
        this.positions = [];
        scene.remove(this.dashedLine);

        if (this.solidLine) {
            scene.remove(this.solidLine);
            this.solidLine.geometry.dispose();

            if (Array.isArray(this.solidLine.material)) {
                this.solidLine.material.forEach((material) => material.dispose());
            } else {
                this.solidLine.material.dispose();
            }

            this.solidLine = null;
        }

        this.render();
    }

    private updateSolidLine() {
        const { scene } = this.context;
        const vertices = [];
        this.positions.forEach((pos) => {
            vertices.push(new Vector3(pos.coordinate.x, pos.coordinate.y, pos.coordinate.z - 0.01));
        });

        if (this.solidLine) {
            this.updateMeshLine(this.solidLine, vertices);
        } else {
            this.solidLine = createToolSolidLine(vertices);
        }
        scene.add(this.solidLine);
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
        const lineWidth = 0.2;
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
        if (this.isInitiation) {
            this.isInitiation = false;
            this.reset();
        }
    };

    private handleMouseMove = async (event: MouseEvent, mouseDownInfo: IMouseDownInfo) => {
        event.preventDefault();
        this.handleMouseMoveNotDragging(event, 'End');
        const { scene } = this.context;
        const lastPosition = this.positions[this.positions.length - 1];
        if (lastPosition) {
            const endPosition = this.computeRaycasterIntersects(event.clientX, event.clientY);
            if (endPosition) {
                const vertices = [lastPosition.coordinate, endPosition];
                this.updateDashedLine(vertices);
                scene.add(this.dashedLine);
                await this.render();
            }
        }
    };

    private handleMouseUp = async (event: MouseEvent, interaction: ContinuousMouseInteractionType) => {
        const { scene, coordinates } = this.context;
        const planeIntersect = this.computeRaycasterIntersects(event.clientX, event.clientY);

        if (interaction === 'click') {
            if (planeIntersect) {
                const position = {
                    coordinate: planeIntersect,
                    direction: 0,
                };
                this.positions.push(position);
            }
        } else if (interaction === 'doubleClick' || interaction === 'rightClick') {
            if (interaction === 'doubleClick') {
                if (this.positions.length > 2) {
                    const position = this.positions.pop();
                    position.instance.remove();
                }
            }
            this.isInitiation = true;
            scene.remove(this.dashedLine);
            await this.copyMessage(
                this.positions
                    .map((position) => coordinates.applyOffset(position.coordinate, true))
                    .map((point) => `(${point.x},${point.y})`)
                    .join('\n'),
            );
        }
        this.updateSolidLine();
        await this.render();
    };

    async render() {
        const { renderer, camera, scene } = this.context;
        const position = this.positions[this.positions.length - 1];

        if (position) {
            const copyCoordinate = position.coordinate.clone().add(new Vector3(0, 0, 0.01));

            // 如果已经存在point,则更新当前point， 否则创建新的point
            if (!position.instance) {
                position.instance = new Point(copyCoordinate, position.direction, {
                    ...this.context,
                    marker: this.marker?.clone(),
                })
                    .setLabelVisible(true)
                    .render();
            } else {
                position.instance.updatePosition(position.coordinate).render();
            }
        }
        renderer.render(scene, camera);
    }

    get positionsCount() {
        return this.positions.length;
    }

    get initiationMarkerPosition() {
        const position = this.positions[this.positions.length - 1];
        return [position?.coordinate, position.direction];
    }

    get initiationMarkerPositionAfterOffset() {
        const { coordinates } = this.context;
        const position = this.positions[this.positions.length - 1];
        return [coordinates.applyOffset(position.coordinate), position.direction];
    }
}
