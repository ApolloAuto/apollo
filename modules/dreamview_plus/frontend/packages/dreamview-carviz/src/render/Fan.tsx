import {
    BufferGeometry,
    CircleGeometry,
    Color,
    Line,
    Material,
    Mesh,
    MeshBasicMaterial,
    Vector2,
    Vector3,
} from 'three';
import { MeshLine, MeshLineMaterial } from 'three.meshline';
import { CSS2DObject } from 'three/examples/jsm/renderers/CSS2DRenderer';
import { createRoot, Root } from 'react-dom/client';
import { IThreeContext } from './type';
import AngleLabel from '../labels/AngleLabel';

interface IFanThreeContext extends IThreeContext {
    fanColor?: number;
    radius?: number;
    lineWidth?: number;
    borderType?: string;
    borderColor?: Color | string | number;
    borderTransparent?: boolean;
    borderOpacity?: number;
    dashSize?: number;
    depthTest?: boolean;
    lineMaterial?: Material;
    borderWidth?: number;
}

export default class FanMarker {
    private fan?: Mesh;

    private roots = new Map<Element, Root>();

    private fanLabel?: CSS2DObject;

    private border?: Line;

    private radius = 4;

    private startPoint: Vector3 = new Vector3();

    private midPoint: Vector3 = new Vector3();

    private endPoint: Vector3 = new Vector3();

    constructor(private context: IFanThreeContext) {
        this.createFan();
        this.createBorder();
    }

    public updatePoints(startPoint: Vector3, midPoint: Vector3, endPoint: Vector3) {
        this.startPoint.copy(startPoint);
        this.midPoint.copy(midPoint);
        this.endPoint.copy(endPoint);

        this.updateFan();
        this.updateBorder();

        return this;
    }

    private calculateAngles() {
        const startVec = this.startPoint.clone().sub(this.midPoint);
        const endVec = this.endPoint.clone().sub(this.midPoint);
        return {
            degree: startVec.angleTo(endVec),
            startAngle: this.getStartTheta(startVec, endVec),
            center: this.midPoint,
            startVec,
            endVec,
        };
    }

    getStartTheta(vec1: Vector3, vec2: Vector3) {
        const cross = new Vector3().crossVectors(vec1, vec2);
        let theta1 = Math.atan(vec1.y / vec1.x) || 0;
        let theta2 = Math.atan(vec2.y / vec2.x) || 0;

        if (cross.z > 0) {
            if ((vec1.x <= 0 && vec1.y >= 0) || (vec1.x <= 0 && vec1.y <= 0)) {
                theta1 += Math.PI;
            }
            return theta1;
        }
        if ((vec2.x <= 0 && vec2.y >= 0) || (vec2.x <= 0 && vec2.y <= 0)) {
            theta2 += Math.PI;
        }
        return theta2;
    }

    private createFan() {
        const { scene, radius } = this.context;
        const angles = this.calculateAngles();
        const geometry = new CircleGeometry(radius || this.radius, 32, angles.startAngle, angles.degree);
        const material = new MeshBasicMaterial({
            color: this.context.fanColor,
            transparent: true,
            opacity: 0.2,
            depthTest: false,
        });
        this.fan = new Mesh(geometry, material);
        this.fan.position.copy(angles.center);

        this.fanLabel = this.createOrUpdateLabel(angles.degree * (180 / Math.PI), angles.center);

        this.fan.add(this.fanLabel);

        scene.add(this.fan);
    }

    private updateFan() {
        if (this.fan) {
            const angles = this.calculateAngles();
            this.fan.geometry = new CircleGeometry(
                this.context.radius || this.radius,
                32,
                angles.startAngle,
                angles.degree,
            );
            this.fan.position.copy(angles.center);
            this.createOrUpdateLabel(angles.degree * (180 / Math.PI), angles.center, this.fanLabel.element);
        } else {
            this.createFan();
        }
    }

    private createBorder() {
        const {
            scene,
            radius,
            borderType,
            borderColor = 0x000000,
            borderTransparent = false,
            borderOpacity = 1,
            dashSize = 0.1,
            depthTest = false,
            borderWidth = 0.2,
        } = this.context;

        const angles = this.calculateAngles();

        // 适配边框宽度
        const adaptRadius = radius || this.radius + borderWidth / 2;
        const adaptStartAngle = angles.startAngle + 0.01;
        const adaptDegree = angles.degree + 0.01;

        const geometry = new CircleGeometry(adaptRadius, 64, adaptStartAngle, adaptDegree);
        geometry.deleteAttribute('normal');
        geometry.deleteAttribute('uv');

        // 获取扇形弧线上的顶点（排除第一个和最后一个顶点，它们是半径的顶点）
        const vertices = geometry.attributes.position.array;
        const segments = [];
        for (let i = 3; i < vertices.length - 3; i += 3) {
            segments.push(new Vector3(vertices[i], vertices[i + 1], vertices[i + 2]));
        }

        const lineGeometry = new BufferGeometry().setFromPoints(segments);

        // 将弧线移动到angles.center的位置
        lineGeometry.translate(angles.center.x, angles.center.y, angles.center.z);

        const commonMaterialOptions = {
            color: borderColor,
            transparent: borderTransparent,
            opacity: borderOpacity,
            depthTest,
        };

        let edgeMaterial;
        let edgeMesh;

        const line = new MeshLine();
        line.setGeometry(lineGeometry);

        if (borderWidth && borderWidth > 0) {
            edgeMaterial = new MeshLineMaterial({
                ...commonMaterialOptions,
                lineWidth: borderWidth,
                sizeAttenuation: true,
                dashArray: borderType === 'dashed' ? dashSize : 0,
                resolution: new Vector2(window.innerWidth, window.innerHeight),
                alphaTest: 0.5,
            });

            edgeMesh = new Mesh(line, edgeMaterial);
            this.border = edgeMesh;
            scene.add(edgeMesh);
        } else {
            throw new Error('Border width must be greater than 0');
        }
    }

    private updateBorder() {
        const { scene } = this.context;
        if (this.border) {
            scene.remove(this.border);
            this.createBorder();
        }
    }

    private createOrUpdateLabel(angle, position, existingElement = null) {
        const labelComponent = <AngleLabel angle={angle} />;

        const angles = this.calculateAngles();
        const halfAngle = angles.degree / 2;
        const offset = 1.5; // 距离外弧度偏移量
        const labelRadius = (this.context.radius || this.radius) + offset;

        const labelPosition = new Vector3(
            labelRadius * Math.cos(angles.startAngle + halfAngle),
            labelRadius * Math.sin(angles.startAngle + halfAngle),
            0,
        );

        if (existingElement) {
            let root = this.roots.get(existingElement);
            if (!root) {
                root = createRoot(existingElement);
                this.roots.set(existingElement, root);
            }
            root.render(labelComponent);
            this.fanLabel.position.copy(labelPosition);
            return existingElement;
        }
        const element = document.createElement('div');
        const root = createRoot(element);
        this.roots.set(element, root);
        root.render(labelComponent);
        const label = new CSS2DObject(element);
        label.position.copy(labelPosition);
        return label;
    }

    render() {
        const { renderer, scene, camera, CSS2DRenderer } = this.context;

        renderer.render(scene, camera);
        CSS2DRenderer.render(scene, camera);
        return this;
    }

    public remove() {
        const { scene } = this.context;

        if (this.fanLabel) {
            this.fan.remove(this.fanLabel);
        }

        if (this.fan) {
            scene.remove(this.fan);
        }
        if (this.border) {
            scene.remove(this.border);
        }
        this.render();
    }
}
