import * as THREE from 'three';
import { union } from 'lodash';
import { MeshLine, MeshLineMaterial } from 'three.meshline';
import { planningParams } from '../constant/params';
import { disposeMesh, disposeGroup, drawCircle, drawArrow } from '../utils/common';
import { drawDashedLineFromPoints, drawSegmentsFromPoints } from '../utils/line';

const DEFAULT_WIDTH = planningParams.defaults.width;
const MIN_INTERVAL = planningParams.minInterval;

function normalizePlanningTrajectory(trajectory) {
    const result = [];
    if (!trajectory || trajectory.length === 0) {
        return [];
    }

    for (let i = 0; i < trajectory.length; i += 1) {
        const point = new THREE.Vector2(trajectory[i].x, trajectory[i].y);
        if (!point) {
            continue;
        }

        if (result.length > 0) {
            const lastPoint = result[result.length - 1];
            const distance = Math.abs(lastPoint.x - point.x) + Math.abs(lastPoint.y - point.y);
            if (distance < MIN_INTERVAL) {
                continue;
            }
        }
        result.push(point);
    }
    if (result.length < 2) {
        return [];
    }

    return result;
}

function drawPullOverBox({ lengthFront, lengthBack, widthLeft, widthRight }) {
    const pullOverStatus = new THREE.Group();
    const color = 0x006aff;
    const polygon = drawSegmentsFromPoints(
        [
            new THREE.Vector3(lengthFront, -widthLeft, 0),
            new THREE.Vector3(lengthFront, widthRight, 0),
            new THREE.Vector3(-lengthBack, widthRight, 0),
            new THREE.Vector3(-lengthBack, -widthLeft, 0),
            new THREE.Vector3(lengthFront, -widthLeft, 0),
        ],
        {
            color,
            linewidth: 2,
            zOffset: 0,
        },
    );
    pullOverStatus.add(polygon);

    const material = new THREE.MeshBasicMaterial({
        color,
        transparent: false,
        opacity: 0.5,
    });
    const circle = drawCircle(0.2, material);
    pullOverStatus.add(circle);

    const heading = drawArrow(color, 1.5, 0.5, 0.5);
    heading.position.z = 0;
    heading.material.linewidth = 2;
    // heading.rotation.set(0, 0, -Math.PI / 2);
    pullOverStatus.add(heading);

    return pullOverStatus;
}

export default class Planning {
    private scene;

    private paths;

    private pathsGeometry;

    private pathsMeshLine;

    private option;

    private oldOptions;

    private coordinates;

    private pullOverBox;

    private lastPullOver;

    private dashLineNames;

    constructor(scene, option, coordinates) {
        this.paths = {};
        this.scene = scene;
        this.option = option;
        this.oldOptions = {};
        this.coordinates = coordinates;
        this.pathsGeometry = {};
        this.pathsMeshLine = {};
        this.pullOverBox = null;
        this.lastPullOver = {};
        this.dashLineNames = [
            'planning_path_boundary_1_regular/self',
            'planning_path_boundary_2_regular/self',
            'planning_path_boundary_1_fallback/self',
            'planning_path_boundary_2_fallback/self',
        ];
    }

    update(planningTrajectory, planningData, autoDrivingCar) {
        if (!this.coordinates.isInitialized()) {
            return;
        }

        // update draw pullOver
        this.updatePullOver(planningData);

        let width = null;
        if (!autoDrivingCar?.width) {
            console.warn(
                `Unable to get the auto driving car's width, planning line width has been set to default: ${DEFAULT_WIDTH} m.`,
            );
            width = DEFAULT_WIDTH;
        } else {
            width = autoDrivingCar.width;
        }

        const newPaths: any = {};

        if (planningTrajectory && planningTrajectory.length) {
            newPaths.trajectory = planningTrajectory.map((point) => ({ x: point.positionX, y: point.positionY }));
        }
        if (planningData && planningData.path) {
            planningData.path?.forEach((path) => {
                if (path.pathPoint?.length) {
                    newPaths[path.name] = path.pathPoint;
                }
            });
        }

        const allPaths = union(Object.keys(this.paths), Object.keys(newPaths));

        allPaths.forEach((name) => {
            let property = planningParams.pathProperties[name];

            if (!property) {
                property = planningParams.pathProperties.default;
            }

            if (newPaths[name]) {
                let points = normalizePlanningTrajectory(newPaths[name]);

                points = this.coordinates.applyOffsetToArray(points);

                if (points.length === 0) {
                    return;
                }

                if (!this.paths[name] || this.dashLineNames.includes(name)) {
                    if (property.style === 'dash') {
                        // dashed lines Redraw, clear old dashed lines before drawing
                        const mesh = this.paths[name];
                        disposeMesh(mesh);
                        this.scene.remove(mesh);
                        this.paths[name] = drawDashedLineFromPoints(points, {
                            color: property.color,
                            linewidth: width * property.width,
                            dashSize: 1,
                            gapSize: 1,
                            zOffset: property.zOffset,
                            opacity: property.opacity,
                            matrixAutoUpdate: true,
                        });
                        this.paths[name].position.z = property.zOffset;
                        this.paths[name].renderOrder = property.renderOrder;
                    } else {
                        const planningGeometry = new THREE.BufferGeometry().setFromPoints(points);
                        const planningMeshLine = new MeshLine();
                        planningMeshLine.setGeometry(planningGeometry);
                        const material = new MeshLineMaterial({
                            color: property.color,
                            opacity: property.opacity,
                            lineWidth: width * property.width,
                        });
                        material.depthTest = true;
                        material.transparent = true;
                        material.side = THREE.DoubleSide;

                        this.pathsGeometry[name] = planningGeometry;
                        this.pathsMeshLine[name] = planningMeshLine;
                        this.paths[name] = new THREE.Mesh(planningMeshLine, material);
                        this.paths[name].position.z = property.zOffset;
                        this.paths[name].renderOrder = property.renderOrder;
                    }
                    this.scene.add(this.paths[name]);
                } else {
                    if (property.style === 'dash') {
                        // Abandon updating points and redraw
                        // this.paths[name].geometry.setFromPoints(points);
                    } else {
                        // update geometry
                        this.pathsGeometry[name].setFromPoints(points);
                        // update MeshLine
                        this.pathsMeshLine[name].setGeometry(this.pathsGeometry[name]);
                    }
                    this.paths[name].geometry.attributes.position.needsUpdate = true;
                    this.paths[name].position.z = property.zOffset;
                    this.paths[name].renderOrder = property.renderOrder;
                }
            } else {
                const mesh = this.paths[name];
                disposeMesh(mesh);
                this.scene.remove(mesh);
                delete this.paths[name];
            }
        });
    }

    dispose() {
        Object.keys(this.paths).forEach((key) => {
            const mesh = this.paths[key];
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });
        this.paths = {};

        if (this.pullOverBox) {
            disposeGroup(this.pullOverBox);
            this.scene.remove(this.pullOverBox);
            this.pullOverBox = null;
        }
        this.lastPullOver = {};
    }

    updatePullOver(planningData) {
        if (!planningData || !planningData.pullOver) {
            if (this.pullOverBox && this.pullOverBox.visible) {
                this.pullOverBox.visible = false;
            }
            return;
        }
        const pullOverData = planningData.pullOver;
        const isNewDimension =
            pullOverData.lengthFront !== this.lastPullOver.lengthFront ||
            pullOverData.lengthBack !== this.lastPullOver.lengthBack ||
            pullOverData.widthLeft !== this.lastPullOver.widthLeft ||
            pullOverData.widthRight !== this.lastPullOver.widthRight;

        if (!this.pullOverBox) {
            this.pullOverBox = drawPullOverBox(pullOverData);
            this.scene.add(this.pullOverBox);
        } else {
            if (isNewDimension) {
                disposeGroup(this.pullOverBox);
                this.scene.remove(this.pullOverBox);
                // redraw
                this.pullOverBox = drawPullOverBox(pullOverData);
                this.scene.add(this.pullOverBox);
            }
            this.pullOverBox.visible = true;
        }
        this.lastPullOver = pullOverData;

        // Set position and theta
        const position = this.coordinates.applyOffset({
            x: pullOverData.position.x,
            y: pullOverData.position.y,
            z: 0.3,
        });
        this.pullOverBox.position.set(position.x, position.y, position.z);
        this.pullOverBox.rotation.set(0, 0, pullOverData.theta);
    }
}
