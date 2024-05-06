import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import * as THREE from 'three';
import { without } from 'lodash';
import stopSignMaterial from '../../../assets/models/stop_sign.mtl';
import stopSignObj from '../../../assets/models/stop_sign.obj';
import { drawStopLine } from '../../utils/stopLine';
import { disposeMesh } from '../../utils/common';
import { getHeadingFromStopLine } from '../../utils/signal';

const STOP_SIGN_SCALE = 0.01;
export default class StopSign {
    private stopSignTemplate;

    private scene;

    private stopLineMeshs;

    private stopSignMeshs;

    private currentStopSignalIds;

    private coordinates;

    private colors;

    constructor(scene, coordinates, colors?) {
        this.colors = colors;
        this.stopSignTemplate = null;
        this.scene = scene;
        this.coordinates = coordinates;
        this.stopLineMeshs = {};
        this.stopSignMeshs = {};
        this.currentStopSignalIds = [];
        this.initStopSignTemplate();
    }

    getPositionAndHeading(stopSign) {
        if (!stopSign?.stopLine?.[0]) {
            return {};
        }
        const heading = getHeadingFromStopLine(stopSign.stopLine[0]);

        if (!Number.isNaN(heading)) {
            const points = stopSign.stopLine?.[0].segment?.[0].lineSegment?.point;
            const length = points.length;
            const stopLinePoint = points[length - 1];
            let position = new THREE.Vector3(stopLinePoint.x, stopLinePoint.y, 0);
            position = this.coordinates.applyOffset(position);
            return { position, heading };
        }
        return {};
    }

    initStopSignTemplate() {
        const mtlLoader = new MTLLoader();
        const objLoader = new OBJLoader();
        mtlLoader.load(stopSignMaterial, (material) => {
            material.preload();
            objLoader.setMaterials(material);
            objLoader.load(stopSignObj, (object) => {
                object.rotateX(Math.PI / 2);
                this.stopSignTemplate = object;
            });
        });
    }

    drawStopSigns(stopSigns) {
        this.currentStopSignalIds = [];
        if (!this.stopSignTemplate) {
            return;
        }
        if (!stopSigns || stopSigns.length === 0) {
            this.dispose();
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        stopSigns.forEach((stopSign) => {
            const id = stopSign.id.id;
            this.currentStopSignalIds.push(id);
            if (this.stopSignMeshs[id]) {
                return;
            }

            const mesh = this.stopSignTemplate.clone();
            const { position, heading } = this.getPositionAndHeading(stopSign);
            if (!position) {
                return;
            }

            mesh.rotation.y = heading || 0;
            mesh.position.set(position.x, position.y, 0);
            mesh.scale.set(STOP_SIGN_SCALE, STOP_SIGN_SCALE, STOP_SIGN_SCALE);
            this.stopSignMeshs[id] = mesh;
            this.scene.add(mesh);

            const stopLine = stopSign.stopLine;
            if (stopLine) {
                stopLine.forEach((item) => {
                    const meshs = drawStopLine(item, this.coordinates, this.colors?.colorMapping.PURE_WHITE);
                    meshs.forEach((mesh) => {
                        this.stopLineMeshs[id] = this.stopLineMeshs[id] || [];
                        this.stopLineMeshs[id].push(mesh);
                        this.scene.add(mesh);
                    });
                });
            }
        });
        this.removeOldTrafficSignals();
    }

    dispose() {
        Object.values(this.stopSignMeshs).forEach((mesh) => {
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });

        Object.keys(this.stopLineMeshs).forEach((id) => {
            const meshArr = this.stopLineMeshs[id];
            meshArr.forEach((item) => {
                disposeMesh(item);
                this.scene.remove(item);
            });
        });

        this.stopLineMeshs = {};
        this.stopSignMeshs = {};
        this.currentStopSignalIds = [];
    }

    removeOldTrafficSignals() {
        const drawedStopSignalIds = Object.keys(this.stopSignMeshs);
        const needRemovedStopSignals = without(drawedStopSignalIds, ...this.currentStopSignalIds);
        if (needRemovedStopSignals && needRemovedStopSignals.length) {
            needRemovedStopSignals.forEach((id) => {
                const removedSignal = this.stopSignMeshs[id];
                disposeMesh(removedSignal);
                this.scene.remove(removedSignal);
                delete this.stopSignMeshs[id];

                const stopLineMeshs = this.stopLineMeshs[id];
                stopLineMeshs.forEach((item) => {
                    disposeMesh(item);
                    this.scene.remove(item);
                });
                delete this.stopLineMeshs[id];
            });
        }
    }
}
