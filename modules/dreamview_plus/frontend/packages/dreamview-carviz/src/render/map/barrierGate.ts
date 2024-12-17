import * as THREE from 'three';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { without } from 'lodash';
import { drawStopLine } from '../../utils/stopLine';
import barrierGateMaterial from '../../../assets/models/barrier_gate.mtl';
import barrierGateObject from '../../../assets/models/barrier_gate.obj';
import { disposeMesh } from '../../utils/common';

export default class BarrierGate {
    private barrierGateMaterials;

    private barrierGateBaseMeshTemplate;

    private barrierGateMeshs;

    private currentBarrierGateIds;

    private stopLineMeshs;

    private scene;

    private coordinates;

    private colors;

    constructor(scene, coordinates, colors?) {
        this.barrierGateMaterials = {};
        this.barrierGateBaseMeshTemplate = null;
        this.barrierGateMeshs = {};
        this.currentBarrierGateIds = [];
        this.stopLineMeshs = {};
        this.scene = scene;
        this.coordinates = coordinates;
        this.colors = colors;
        this.initBarrierGateTemplate();
    }

    initBarrierGateTemplate() {
        const mtlLoader = new MTLLoader();
        const objLoader = new OBJLoader();

        mtlLoader.load(barrierGateMaterial, async (materials) => {
            materials.preload();
            this.barrierGateMaterials = materials;
            objLoader.setMaterials(materials);
            objLoader.load(barrierGateObject, (object) => {
                this.barrierGateBaseMeshTemplate = object;
            });
        });
    }

    drawBarrierGates(barrierGates) {
        this.currentBarrierGateIds = [];
        if (!barrierGates || barrierGates.length === 0) {
            this.dispose();
            return;
        }

        if (!this.barrierGateBaseMeshTemplate) {
            return;
        }

        if (!this.coordinates.isInitialized()) {
            return;
        }
        barrierGates.forEach((barrierGate) => {
            const id = barrierGate.id.id;
            this.currentBarrierGateIds.push(id);
            const stopLines = barrierGate.stopLine;

            if (this.barrierGateMeshs?.[id]) {
                return;
            }

            const barrierOffsetPosition = barrierGate.polygon.point.map((point) => this.coordinates.applyOffset(point));

            const barrierCuboidLength = barrierOffsetPosition[0].distanceTo(barrierOffsetPosition[2]);
            // const barrierCuboidWidth = barrierOffsetPosition[2].distanceTo(barrierOffsetPosition[4]);
            // const barrierCuboidHeight = barrierOffsetPosition[0].distanceTo(barrierOffsetPosition[1]);

            // const center = new THREE.Vector3();
            // for (let i = 0; i < barrierOffsetPosition.length; i += 1) {
            //     center.add(barrierOffsetPosition[i]);
            // }
            // center.divideScalar(barrierOffsetPosition.length);
            // const pointGeometry = new THREE.BufferGeometry();
            // pointGeometry.setAttribute(
            //     'position',
            //     new THREE.BufferAttribute(new Float32Array([center.x, center.y, center.z]), 3),
            // );
            // const pointMatrial = new THREE.PointsMaterial({ color: 0xff0000, size: 1 });
            // const centerPoint = new THREE.Points(pointGeometry, pointMatrial);
            // this.scene.add(centerPoint);

            const barrierGateMesh = this.barrierGateBaseMeshTemplate.clone();

            // 5.18是道闸模型杆的长度，计算道闸实际长度和道闸杆的缩放比例
            const scaleFactor = barrierCuboidLength / 5.18;
            // 道闸模型是Group物体，其中childern[0]是道闸基座，children[1]是道闸杆，道闸基座向道闸杆延伸的水平方向为物体x轴正方向
            // 将道闸杆沿着x轴方向缩放至同只是道闸一样的长度
            barrierGateMesh.children[1].scale.set(scaleFactor, 1, 1);
            // 道闸模型的坐标原点为道闸杆不靠近基座终点并且与基座底部平面垂直的位置，即道闸模型的右下角位置
            const scaleXOffset = barrierGateMesh.children[1].position.x + 5.18 * (scaleFactor - 1);
            barrierGateMesh.children[1].position.setX(barrierGateMesh.children[1].position.x + scaleXOffset);

            const barrierGateCenter = new THREE.Vector3()
                .addVectors(barrierOffsetPosition[2], barrierOffsetPosition[4])
                .multiplyScalar(0.5);
            // const barrierGateCenterGeometry = new THREE.BufferGeometry();
            // barrierGateCenterGeometry.setAttribute(
            //     'position',
            //     new THREE.BufferAttribute(
            //         new Float32Array([barrierGateCenter.x, barrierGateCenter.y, barrierGateCenter.z]),
            //         3,
            //     ),
            // );
            // const barrierGateCenterMatrial = new THREE.PointsMaterial({ color: 0x306044, size: 1 });
            // const barrierGateCenterMesh = new THREE.Points(barrierGateCenterGeometry, barrierGateCenterMatrial);
            // this.scene.add(barrierGateCenterMesh);

            barrierGateMesh.position.copy(barrierGateCenter);

            const unitLongEdgeVector = new THREE.Vector3();
            unitLongEdgeVector.subVectors(barrierOffsetPosition[2], barrierOffsetPosition[0]).normalize();
            const quaternion = new THREE.Quaternion();
            quaternion.setFromUnitVectors(new THREE.Vector3(1, 0, 0), unitLongEdgeVector);
            barrierGateMesh.applyQuaternion(quaternion);
            barrierGateMesh.rotateX(Math.PI / 2);

            this.barrierGateMeshs[id] = barrierGateMesh;

            this.scene.add(barrierGateMesh);

            // const barrierVertices = new Float32Array(
            //     barrierGate.polygon.point.reduce((pre, cur) => {
            //         const offsetPoint = this.coordinates.applyOffset(cur);
            //         return pre.concat([offsetPoint.x, offsetPoint.y, offsetPoint.z]);
            //     }, []),
            // );
            // const colors = [
            //     { name: '红色', code: 0xff0000 },
            //     { name: '绿色', code: 0x00ff00 },
            //     { name: '蓝色', code: 0x0000ff },
            //     { name: '黄色', code: 0xffff00 },
            //     { name: '紫罗兰色', code: 0xff00ff },
            //     { name: '青色', code: 0x00ffff },
            //     { name: '粉色', code: 0xff99ff },
            //     { name: '白色', code: 0xffffff },
            // ];
            // for (let i = 0; i < barrierVertices.length / 3; i += 1) {
            //     const pointGeometry = new THREE.BufferGeometry();
            //     pointGeometry.setAttribute(
            //         'position',
            //         new THREE.BufferAttribute(
            //             new Float32Array([
            //                 barrierVertices[i * 3],
            //                 barrierVertices[i * 3 + 1],
            //                 barrierVertices[i * 3 + 2],
            //             ]),
            //             3,
            //         ),
            //     );
            //     const pointMatrial = new THREE.PointsMaterial({ color: colors[i].code, size: 1 });
            //     const point = new THREE.Points(pointGeometry, pointMatrial);
            //     this.scene.add(point);
            // }

            stopLines.forEach((stopLine) => {
                const lines = drawStopLine(stopLine, this.coordinates, this.colors?.colorMapping.PURE_WHITE);
                lines.forEach((line) => {
                    this.stopLineMeshs[id] = this.stopLineMeshs[id] || [];
                    this.stopLineMeshs[id].push(line);
                    this.scene.add(line);
                });
            });
        });
        this.removeOldBarrierGates();
    }

    dispose() {
        Object.values(this.barrierGateMeshs).forEach((barrierGateMesh) => {
            disposeMesh(barrierGateMesh);
            this.scene.remove(barrierGateMesh);
        });
        Object.keys(this.stopLineMeshs).forEach((barrierGateId) => {
            const stopLineAttr = this.stopLineMeshs[barrierGateId];
            stopLineAttr.forEach((stopLine) => {
                disposeMesh(stopLine);
                this.scene.remove(stopLine);
            });
        });
        this.barrierGateMeshs = {};
        this.stopLineMeshs = {};
        this.currentBarrierGateIds = [];
    }

    clearSignalStatus() {
        console.log('clearSignalStatus');
    }

    removeOldBarrierGates() {
        const drawedBarrierGateIds = Object.keys(this.barrierGateMeshs);
        const needRemovedBarrierGates = without(drawedBarrierGateIds, ...this.currentBarrierGateIds);
        if (needRemovedBarrierGates && needRemovedBarrierGates.length) {
            needRemovedBarrierGates.forEach((barrierGateId) => {
                const removeBarrierGate = this.barrierGateMeshs[barrierGateId];
                disposeMesh(removeBarrierGate);
                this.scene.remove(removeBarrierGate);
                delete this.barrierGateMeshs[barrierGateId];

                const removeStopLines = this.stopLineMeshs[barrierGateId];
                removeStopLines.forEach((stopLine) => {
                    disposeMesh(stopLine);
                    this.scene.remove(stopLine);
                });
                delete this.stopLineMeshs[barrierGateId];
            });
        }
    }
}
