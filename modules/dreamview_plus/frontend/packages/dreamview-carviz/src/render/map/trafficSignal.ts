import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { without } from 'lodash';
import { Mesh } from 'three';
import { drawStopLine } from '../../utils/stopLine';
import trafficLightMaterial from '../../../assets/models/traffic_light.mtl';
import trafficLightObject from '../../../assets/models/traffic_light.obj';
import { disposeMesh } from '../../utils/common';
import { getPositionAndHeading } from '../../utils/signal';

const TRAFFIC_LIGHT_SCALE = 2.3;
const SUBSIGNAL_TO_INDEX = {
    GREEN: 6,
    YELLOW: 5,
    RED: 4,
};
const SUBSIGNAL_TO_LIGHT = {
    GREEN: 0x14f470,
    YELLOW: 0xcec832,
    RED: 0xff0000,
};

export default class TrafficSignal {
    private trafficMaterials;

    private stopLineMeshs;

    private trafficBaseMeshTemplate;

    private baseSignalMeshs;

    private signalStatusMeshs;

    private scene;

    private currentTrafficSignalIds;

    private coordinates;

    private colors;

    constructor(scene, coordinates, colors?) {
        this.colors = colors;
        this.trafficBaseMeshTemplate = null;
        this.trafficMaterials = {};
        this.baseSignalMeshs = {};
        this.signalStatusMeshs = [];
        this.currentTrafficSignalIds = [];
        this.stopLineMeshs = {};
        this.scene = scene;
        this.coordinates = coordinates;
        this.initTrafficTemplate();
    }

    initTrafficTemplate() {
        const mtlLoader = new MTLLoader();
        const objLoader = new OBJLoader();
        const getLightMaterial = (originMaterial, subsignal) => {
            const lightMaterial = originMaterial.clone();
            lightMaterial.emissive.set(SUBSIGNAL_TO_LIGHT[subsignal]);
            return lightMaterial;
        };

        mtlLoader.load(trafficLightMaterial, async (materials) => {
            const [, , DARK_GREEN, DARK_YELLOW, DARK_RED] = materials.getAsArray();
            this.trafficMaterials = {
                GREEN: {
                    DARK: DARK_GREEN,
                    LIGHT: getLightMaterial(DARK_GREEN, 'GREEN'),
                },
                YELLOW: {
                    DARK: DARK_YELLOW,
                    LIGHT: getLightMaterial(DARK_YELLOW, 'YELLOW'),
                },
                RED: {
                    DARK: DARK_RED,
                    LIGHT: getLightMaterial(DARK_RED, 'RED'),
                },
            };

            materials.preload();
            objLoader.setMaterials(materials);
            objLoader.load(trafficLightObject, (object) => {
                object.rotation.x = Math.PI / 2;
                this.trafficBaseMeshTemplate = object;
            });
        });
    }

    updateTrafficStatus(signals) {
        if (signals && signals.length !== 0) {
            this.clearSignalStatus();
            const trafficSignalCurMapping = {};
            signals.forEach((signal) => {
                trafficSignalCurMapping[signal.id.id] = signal.currentSignal;
            });

            Object.keys(this.baseSignalMeshs).forEach((id) => {
                if (id in trafficSignalCurMapping) {
                    const mesh = this.baseSignalMeshs[id];
                    const subsignal = trafficSignalCurMapping[id];
                    const index = SUBSIGNAL_TO_INDEX[subsignal];
                    if (index) {
                        const lightMaterial = this.trafficMaterials[subsignal].LIGHT;
                        const subsignalMesh = mesh.children[index];
                        subsignalMesh.material = lightMaterial;
                        subsignalMesh.subsignal = subsignal;
                        this.signalStatusMeshs.push(subsignalMesh);
                    }
                }
            });
        }
    }

    drawTrafficSignals(signals) {
        this.currentTrafficSignalIds = [];
        if (!signals || signals.length === 0) {
            this.dispose();
            return;
        }
        if (!this.trafficBaseMeshTemplate) {
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        signals.forEach((signal) => {
            const id = signal.id.id;
            this.currentTrafficSignalIds.push(id);
            if (this.baseSignalMeshs[id]) {
                return;
            }
            let { position } = getPositionAndHeading(signal);
            const { heading } = getPositionAndHeading(signal);
            if (!position) {
                return;
            }
            position = this.coordinates.applyOffset(position);
            const mesh = this.trafficBaseMeshTemplate.clone();
            mesh.rotation.y = heading || 0;
            mesh.position.set(position.x, position.y, 0);
            mesh.scale.set(TRAFFIC_LIGHT_SCALE, TRAFFIC_LIGHT_SCALE, TRAFFIC_LIGHT_SCALE);
            this.baseSignalMeshs[signal.id.id] = mesh;
            this.scene.add(mesh);

            signal.stopLine.forEach((item) => {
                const lines = drawStopLine(item, this.coordinates, this.colors?.colorMapping.PURE_WHITE);
                lines.forEach((line) => {
                    this.stopLineMeshs[id] = this.stopLineMeshs[id] || [];
                    this.stopLineMeshs[id].push(line);
                    this.scene.add(line);
                });
            });
        });
        this.removeOldTrafficSignals();
    }

    dispose() {
        Object.values(this.baseSignalMeshs).forEach((mesh) => {
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
        this.baseSignalMeshs = {};
        this.stopLineMeshs = {};
        this.currentTrafficSignalIds = [];
    }

    clearSignalStatus() {
        this.signalStatusMeshs.forEach((mesh) => {
            const darkMaterial = this.trafficMaterials[mesh.subsignal].DARK;
            if (darkMaterial) {
                mesh.material = darkMaterial;
            }
        });
        this.signalStatusMeshs = [];
    }

    removeOldTrafficSignals() {
        const drawedTrafficSignalIds = Object.keys(this.baseSignalMeshs);
        const needRemovedTrafficSignals = without(drawedTrafficSignalIds, ...this.currentTrafficSignalIds);
        if (needRemovedTrafficSignals && needRemovedTrafficSignals.length) {
            needRemovedTrafficSignals.forEach((id) => {
                const removedSignal = this.baseSignalMeshs[id];
                disposeMesh(removedSignal);
                this.scene.remove(removedSignal);
                delete this.baseSignalMeshs[id];

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
