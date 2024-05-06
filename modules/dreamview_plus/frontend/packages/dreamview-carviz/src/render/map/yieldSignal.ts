import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import * as THREE from 'three';
import { without } from 'lodash';
import yieldSignMaterial from '../../../assets/models/yield_sign.mtl';
import yieldSignObj from '../../../assets/models/yield_sign.obj';
import { drawStopLine } from '../../utils/stopLine';
import { disposeMesh } from '../../utils/common';
import { getHeadingFromStopLine } from '../../utils/signal';

const STOP_SIGN_SCALE = 0.01;
export default class YieldSignal {
    private yieldSignTemplate;

    private scene;

    private yieldLineMeshs;

    private yieldSignMeshs;

    private currentYieldgnalIds;

    private coordinates;

    private colors;

    constructor(scene, coordinates, colors) {
        this.colors = colors;
        this.yieldSignTemplate = null;
        this.scene = scene;
        this.coordinates = coordinates;
        this.yieldLineMeshs = {};
        this.yieldSignMeshs = {};
        this.currentYieldgnalIds = [];
        this.iniYieldSignTemplate();
    }

    getPositionAndHeading(yieldSign) {
        if (!yieldSign?.stopLine?.[0]) {
            return {};
        }
        const heading = getHeadingFromStopLine(yieldSign.stopLine[0]);

        if (!Number.isNaN(heading)) {
            const points = this.coordinates.applyOffsetToArray(yieldSign.stopLine?.[0].segment?.[0].lineSegment?.point);
            const length = points.length;
            const stopLinePoint = points[length - 1];
            let position = new THREE.Vector3(stopLinePoint.x, stopLinePoint.y, 0);
            position = this.coordinates.applyOffset(position);
            return { position, heading };
        }
        return {};
    }

    iniYieldSignTemplate() {
        const mtlLoader = new MTLLoader();
        const objLoader = new OBJLoader();
        mtlLoader.load(yieldSignMaterial, (material) => {
            material.preload();
            objLoader.setMaterials(material);
            objLoader.load(yieldSignObj, (object) => {
                object.rotateX(Math.PI / 2);
                this.yieldSignTemplate = object;
            });
        });
    }

    drawYieldSigns(yieldSigns) {
        this.currentYieldgnalIds = [];
        if (!this.yieldSignTemplate) {
            return;
        }
        if (!yieldSigns || yieldSigns.length === 0) {
            this.dispose();
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        yieldSigns.forEach((yieldSign) => {
            const id = yieldSign.id.id;
            this.currentYieldgnalIds.push(id);
            if (this.yieldSignMeshs[id]) {
                return;
            }
            const mesh = this.yieldSignTemplate.clone();
            const { position, heading } = this.getPositionAndHeading(yieldSign);
            if (!position) {
                return;
            }

            mesh.rotation.y = heading || 0;
            mesh.position.set(position.x, position.y, 0);
            mesh.scale.set(STOP_SIGN_SCALE, STOP_SIGN_SCALE, STOP_SIGN_SCALE);
            this.yieldSignMeshs[id] = mesh;
            this.scene.add(mesh);

            const stopLine = yieldSign.stopLine;
            if (stopLine) {
                stopLine.forEach((item) => {
                    const meshs = drawStopLine(item, this.coordinates, this.colors?.colorMapping.PURE_WHITE);
                    meshs.forEach((mesh) => {
                        this.yieldLineMeshs[id] = this.yieldLineMeshs[id] || [];
                        this.yieldLineMeshs[id].push(mesh);
                        this.scene.add(mesh);
                    });
                });
            }
        });
        this.removeOldTrafficSignals();
    }

    dispose() {
        Object.values(this.yieldSignMeshs).forEach((mesh) => {
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });

        Object.keys(this.yieldLineMeshs).forEach((id) => {
            const meshArr = this.yieldLineMeshs[id];
            meshArr.forEach((item) => {
                disposeMesh(item);
                this.scene.remove(item);
            });
        });
        this.yieldLineMeshs = {};
        this.yieldSignMeshs = {};
        this.currentYieldgnalIds = [];
    }

    removeOldTrafficSignals() {
        const drawedStopSignalIds = Object.keys(this.yieldSignMeshs);
        const needRemovedStopSignals = without(drawedStopSignalIds, ...this.currentYieldgnalIds);
        if (needRemovedStopSignals && needRemovedStopSignals.length) {
            needRemovedStopSignals.forEach((id) => {
                const removedSignal = this.yieldSignMeshs[id];
                disposeMesh(removedSignal);
                this.scene.remove(removedSignal);
                delete this.yieldSignMeshs[id];

                const stopLineMeshs = this.yieldLineMeshs[id];
                stopLineMeshs.forEach((item) => {
                    disposeMesh(item);
                    this.scene.remove(item);
                });
                delete this.yieldLineMeshs[id];
            });
        }
    }
}
