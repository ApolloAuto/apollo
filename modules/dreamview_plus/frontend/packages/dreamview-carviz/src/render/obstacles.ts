import * as THREE from 'three';
import { camelCase, memoize } from 'lodash';
import { obstacleColorMapping } from '../constant/common';
import { disposeMesh, drawArrow, drawImge, disposeGroup, drawBox, drawDashedBox, drawSolidBox } from '../utils/common';
import iconObjectYield from '../../assets/images/decision/object-yield.png';
import ThreeObjectPool from '../utils/ThreeObjectPool';

const DEFAULT_HEIGHT = 1.5;

enum ObjectType {
    UNKNOWN = 0,
    UNKNOWN_MOVABLE = 1,
    UNKNOWN_UNMOVABLE = 2,
    PEDESTRIAN = 3,
    BICYCLE = 4,
    VEHICLE = 5,
    VIRTUAL = 6,
    CIPV = 7,
}

function getSensorType(key) {
    if (key.search('radar') !== -1) {
        return 'RadarSensor';
    }
    if (key.search('lidar') !== -1 || key.search('velodyne') !== -1) {
        return 'LidarSensor';
    }
    if (key.search('camera') !== -1) {
        return 'CameraSensor';
    }
    return null;
}

const memoizeGetSensorType = memoize(getSensorType);

enum ObjectName {
    ObstacleHeading,
}

enum POOL_TYPE {
    ARROW = 'ARROW',
    BIG_ARROW = 'BIG_ARROW',
    ICON = 'ICON',
    LINE_LOOP = 'LINE_LOOP',
    LINE_DASH = 'LINE_DASH',
}
export default class Obstacles {
    private obstacleMeshs;

    private speedHeadingArrows;

    private obstacleHeadingArrows;

    private textMeshs;

    private iconMeshs;

    private iconMeshTemplate;

    private scene;

    private view;

    private text;

    private solidFaceCubeMeshTemplate;

    private option;

    private coordinates;

    private memo;

    private colors;

    private arrowObjectPool: ThreeObjectPool<THREE.Line>;

    private arrowBigObjectPool: ThreeObjectPool<THREE.Line>;

    private iconObjectPool: ThreeObjectPool<THREE.Mesh>;

    private LinePoolMemo = new Map();

    private LineDashMemo = new Map();

    constructor(scene, view, text, option, coordinates, colors?) {
        this.colors = colors;
        this.memo = new Map();
        this.scene = scene;
        this.view = view;
        this.text = text;
        this.option = option;
        this.coordinates = coordinates;
        this.obstacleMeshs = [];
        this.speedHeadingArrows = [];
        this.obstacleHeadingArrows = [];
        this.textMeshs = [];
        this.iconMeshs = [];
        this.iconMeshTemplate = null;
        this.solidFaceCubeMeshTemplate = null;
        this.drawCubeTemplate();
        this.arrowObjectPool = new ThreeObjectPool({
            type: POOL_TYPE.ARROW,
            syncFactory: () => drawArrow(0xffffff),
            dispose: (object) => (object.visible = false),
            reset: (object) => (object.visible = true),
        });
        this.arrowBigObjectPool = new ThreeObjectPool({
            type: POOL_TYPE.BIG_ARROW,
            syncFactory: () => drawArrow(0xffffff),
            dispose: (object) => (object.visible = false),
            reset: (object) => (object.visible = true),
        });
        this.iconObjectPool = new ThreeObjectPool({
            type: POOL_TYPE.ICON,
            syncFactory: () => drawImge(iconObjectYield, 1, 1),
            dispose: (object) => (object.visible = false),
            reset: (object) => (object.visible = true),
        });
    }

    autoLinePoolInstance(color) {
        const poolInstance = this.LinePoolMemo.get(color);
        if (poolInstance) {
            return poolInstance;
        }
        this.LinePoolMemo.set(
            color,
            new ThreeObjectPool({
                type: POOL_TYPE.LINE_LOOP,
                syncFactory: () =>
                    new THREE.LineLoop(
                        new THREE.BufferGeometry(),
                        this.getStaticColorMaterial(color, () => {
                            return new THREE.LineBasicMaterial({
                                color,
                            });
                        }),
                    ),
                dispose: (object) => (object.visible = false),
                reset: (object) => (object.visible = true),
            }),
        );
        return this.LinePoolMemo.get(color);
    }

    autoLineSegmentsPoolInstance(color) {
        const poolInstance = this.LineDashMemo.get(color);
        if (poolInstance) {
            return poolInstance;
        }
        this.LineDashMemo.set(
            color,
            new ThreeObjectPool({
                type: POOL_TYPE.LINE_DASH,
                syncFactory: () =>
                    new THREE.LineSegments(
                        new THREE.BufferGeometry(),
                        this.getStaticDashMaterial(color, () => {
                            return new THREE.LineDashedMaterial({ color });
                        }),
                    ),
                dispose: (object) => (object.visible = false),
                reset: (object) => (object.visible = true),
            }),
        );
        return this.LineDashMemo.get(color);
    }

    getMemo(id, sceneName, init) {
        const scene = this.memo.get(sceneName);
        if (!scene) {
            this.memo.set(sceneName, new Map());
        }
        const memo = this.memo.get(sceneName).get(id);
        if (memo) {
            return memo;
        }
        const val = init();
        val.memoName = `${sceneName}_${id}`;
        this.memo.get(sceneName).set(id, val);
        return this.memo.get(sceneName).get(id);
    }

    getTrafficConeMemo(id, init) {
        return this.getMemo(id, 'trafficConeMemo', init);
    }

    getStaticColorMaterial(id, init) {
        return this.getMemo(id, 'staticColorMateria', init);
    }

    getStaticDashMaterial(id, init) {
        return this.getMemo(id, 'staticDashMateria', init);
    }

    drawObstacleHeading(obstacle) {
        const { height, positionX, positionY, heading } = obstacle;
        const position = this.coordinates.applyOffset({ x: positionX, y: positionY });
        const color = 0xffffff;
        const arrowMesh = this.arrowObjectPool.acquireSync();
        arrowMesh.rotation.z = heading;
        arrowMesh.material.color.setHex(color);
        arrowMesh.position.set(position.x, position.y, (height || DEFAULT_HEIGHT) / 2);
        return arrowMesh;
    }

    drawSpeedHeading(obstacle) {
        const { height, positionX, positionY, type, speedHeading, speed, id } = obstacle;
        const position = this.coordinates.applyOffset({ x: positionX, y: positionY });
        const color = this.colors.obstacleColorMapping[type] || this.colors.obstacleColorMapping.DEFAULT;
        const arrowMesh = this.arrowBigObjectPool.acquireSync();
        arrowMesh.material.color.setHex(color);
        arrowMesh.rotation.z = speedHeading;
        const scale = 1 + Math.log2(speed);
        arrowMesh.scale.set(scale, scale, scale);
        arrowMesh.position.set(position.x, position.y, (height || DEFAULT_HEIGHT) / 2);
        return arrowMesh;
    }

    drawTrafficCone(obstacle) {
        const { positionX, positionY, positionZ, id } = obstacle;
        const position = this.coordinates.applyOffset({ x: positionX, y: positionY });
        const mesh = this.getTrafficConeMemo(id, () => {
            const geometry = new THREE.CylinderGeometry(0.1, 0.25, 0.914, 32);
            const material = new THREE.MeshBasicMaterial({
                color: this.colors.obstacleColorMapping.TRAFFICCONE,
                transparent: true,
                opacity: 0.64,
            });
            return new THREE.Mesh(geometry, material);
        });
        mesh.rotation.set(Math.PI / 2, 0, 0);
        // todo z的值时否要改成 0.914 / 2
        mesh.position.set(position.x, position.y, positionZ);
        return mesh;
    }

    update(obstacles, measurements, autoDrivingCar) {
        this.dispose();
        this.updateObstacles(obstacles, autoDrivingCar);
        this.updateSensorMeasurements(measurements);
    }

    updateObstacles(obstacles, autoDrivingCar) {
        if (!this.coordinates.isInitialized()) {
            return;
        }
        const { obstacleHeading, obstacleDistanceAndSpeed } = this.option.layerOption.Perception;
        // for (let i = 0; i < obstacles.length; i += 1) {
        for (let i = 0; i < obstacles.length; i += 1) {
            const obstacle = obstacles[i];
            const { positionX, positionY, yieldedobstacle, type, id } = obstacle;
            if (!positionX || !positionY) {
                continue;
            }
            if (!this.option.layerOption.Perception[camelCase(type)]) {
                continue;
            }

            const mesh = this.drawObstacle(obstacle);
            if (!mesh) {
                return;
            }
            if (mesh) {
                // 判断mesh是否在this.scene中
                if (this.scene.children.indexOf(mesh) === -1) {
                    this.scene.add(mesh);
                }
                this.obstacleMeshs.push(mesh);
            }

            if (obstacleHeading) {
                const obstacleHeadingArrow = this.drawObstacleHeading(obstacle);
                if (obstacleHeadingArrow) {
                    this.obstacleHeadingArrows.push(obstacleHeadingArrow);
                    if (this.scene.children.indexOf(obstacleHeadingArrow) === -1) {
                        this.scene.add(obstacleHeadingArrow);
                    }
                }
            }

            if (obstacleDistanceAndSpeed) {
                const speedHeadingArrow = this.drawSpeedHeading(obstacle);
                if (speedHeadingArrow) {
                    this.speedHeadingArrows.push(speedHeadingArrow);
                    if (this.scene.children.indexOf(speedHeadingArrow) === -1) {
                        this.scene.add(speedHeadingArrow);
                    }
                }
            }
            if (yieldedobstacle) {
                const icon = this.iconObjectPool.acquireSync();
                const position = this.coordinates.applyOffset({
                    x: positionX,
                    y: positionY,
                    z: (obstacle.height || DEFAULT_HEIGHT) + 0.5,
                });
                icon.position.set(position.x, position.y, position.z);
                if (this.scene.children.indexOf(icon) === -1) {
                    this.scene.add(icon);
                }
                this.iconMeshs.push(icon);
            }
            const texts = this.drawTexts(obstacle, autoDrivingCar);
            texts.forEach((mesh) => {
                this.textMeshs.push(mesh);
                this.scene.add(mesh);
            });
        }
    }

    updateSensorMeasurements(sensorMeasurements) {
        if (!this.coordinates.isInitialized()) {
            return;
        }
        const { lidar, radar, camera, obstacleHeading } = this.option.layerOption.Perception;
        if (!lidar && !radar && !camera) {
            return;
        }
        if (!sensorMeasurements) {
            return;
        }
        Object.keys(sensorMeasurements).forEach((key) => {
            const sensorType = memoizeGetSensorType(key.toLowerCase());
            if (!sensorType || !this.option.layerOption.Perception[sensorType]) {
                return;
            }
            const measurements = sensorMeasurements[key]?.sensorMeasurement;
            if (!measurements || measurements.length === 0) {
                return;
            }
            measurements.forEach((item) => {
                if (!item.positionX || !item.positionY) {
                    return;
                }
                if (obstacleHeading) {
                    const obstacleHeadingArrow = this.drawObstacleHeading(item);
                    this.obstacleHeadingArrows.push(obstacleHeadingArrow);
                    this.scene.add(obstacleHeadingArrow);
                }
                const mesh = this.drawObstacle(item);
                if (mesh) {
                    this.scene.add(mesh);
                    this.obstacleMeshs.push(mesh);
                }
            });
        });
    }

    releasePool(mesh) {
        const type = mesh?.userData?.type;
        if (type === POOL_TYPE.ARROW) {
            this.arrowObjectPool.release(mesh);
        } else if (type === POOL_TYPE.BIG_ARROW) {
            this.arrowBigObjectPool.release(mesh);
        } else if (type === POOL_TYPE.ICON) {
            this.iconObjectPool.release(mesh);
        } else if (type === POOL_TYPE.LINE_DASH) {
            this.LineDashMemo.get(mesh.material.color).release(mesh);
        } else if (type === POOL_TYPE.LINE_LOOP) {
            this.LinePoolMemo.get(mesh.material.color).release(mesh);
        }
    }

    dispose() {
        const disposeMeshArray = (meshArray) => {
            for (let i = 0; i < meshArray.length; i++) {
                const mesh = meshArray[i];
                if (mesh.type === 'Object3D') {
                    disposeGroup(mesh);
                    this.scene.remove(mesh);
                } else if (mesh.type === 'Group') {
                    disposeGroup(mesh);
                    this.scene.remove(mesh);
                    mesh.traverse((child) => {
                        if (mesh?.userData?.type) {
                            this.releasePool(mesh);
                        } else {
                            disposeMesh(child);
                        }
                    });
                } else if (mesh?.userData?.type) {
                    this.releasePool(mesh);
                } else {
                    disposeMesh(mesh);
                    this.scene.remove(mesh);
                }
            }
            meshArray.length = 0;
        };

        disposeMeshArray(this.obstacleHeadingArrows);
        disposeMeshArray(this.speedHeadingArrows);
        disposeMeshArray(this.obstacleMeshs);
        disposeMeshArray(this.textMeshs);
        disposeMeshArray(this.iconMeshs);

        this.obstacleHeadingArrows.length = 0;
        this.speedHeadingArrows.length = 0;
        this.obstacleMeshs.length = 0;
        this.textMeshs.length = 0;
        this.iconMeshs.length = 0;

        // 重置相关对象
        this.text.reset();
    }

    drawCubeTemplate() {
        const color = this.colors.obstacleColorMapping.DEFAULT;
        const solidFaceCube = drawSolidBox(1, 1, 1, color);
        this.solidFaceCubeMeshTemplate = solidFaceCube;
    }

    drawObstaclePolygon(obstacle) {
        const { polygonPoint, height, confidence, source, type, id } = obstacle;
        const bottomPoints = this.coordinates.applyOffsetToArray(polygonPoint);
        const topPoints = bottomPoints.map((point) => ({
            x: point.x,
            y: point.y,
            z: point.z + (source === 'v2x' ? height : confidence < 1 ? confidence * height : height),
        }));
        const color = this.colors.obstacleColorMapping[type] || this.colors.obstacleColorMapping.DEFAULT;

        const group = new THREE.Group();

        const geometry = new THREE.BufferGeometry();
        const positions = [];
        // 绘制底面
        bottomPoints.forEach((point, index) => {
            positions.push(point.x, point.y, point.z);
            if (index > 0) {
                positions.push(bottomPoints[index - 1].x, bottomPoints[index - 1].y, bottomPoints[index - 1].z);
                positions.push(point.x, point.y, point.z);
            }
        });
        positions.push(
            bottomPoints[bottomPoints.length - 1].x,
            bottomPoints[bottomPoints.length - 1].y,
            bottomPoints[bottomPoints.length - 1].z,
        );
        positions.push(bottomPoints[0].x, bottomPoints[0].y, bottomPoints[0].z);

        // 绘制顶面
        topPoints.forEach((point, index) => {
            positions.push(point.x, point.y, point.z);
            if (index > 0) {
                positions.push(topPoints[index - 1].x, topPoints[index - 1].y, topPoints[index - 1].z);
                positions.push(point.x, point.y, point.z);
            }
        });
        positions.push(
            topPoints[topPoints.length - 1].x,
            topPoints[topPoints.length - 1].y,
            topPoints[topPoints.length - 1].z,
        );
        positions.push(topPoints[0].x, topPoints[0].y, topPoints[0].z);

        // 连接底面和顶面
        bottomPoints.forEach((bottomPoint, index) => {
            const topPoint = topPoints[index];
            positions.push(bottomPoint.x, bottomPoint.y, bottomPoint.z);
            positions.push(topPoint.x, topPoint.y, topPoint.z);
        });

        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        const material = new THREE.LineBasicMaterial({ color });
        const lineSegments = new THREE.LineSegments(geometry, material);
        group.add(lineSegments);

        return group;
    }

    drawObstacle(obstacle) {
        const { polygonPoint, length, width, height, source } = obstacle;
        const isV2x = source === 'v2x';
        let mesh = null;
        if (obstacle.subType === 'ST_TRAFFICCONE') {
            mesh = this.drawTrafficCone(obstacle);
        } else if (polygonPoint && polygonPoint.length > 0 && this.option.layerOption.Perception.polygon) {
            mesh = this.drawObstaclePolygon(obstacle);
        } else if (length && width && height && this.option.layerOption.Perception.boundingbox) {
            if (isV2x) {
                mesh = this.drawV2xCube(obstacle);
            } else {
                mesh = this.drawCube(obstacle);
            }
        }
        return mesh;
    }

    drawV2xCube(obstacle) {
        const { length, width, height, positionX, positionY, type, heading, id } = obstacle;
        const color = this.colors.obstacleColorMapping[type] || this.colors.obstacleColorMapping.DEFAULT;
        const v2XCubeMesh = this.solidFaceCubeMeshTemplate.clone();
        const position = this.coordinates.applyOffset({
            x: positionX,
            y: positionY,
            z: (obstacle.height || DEFAULT_HEIGHT) / 2,
        });
        v2XCubeMesh.scale.set(length, width, height);
        v2XCubeMesh.position.set(position.x, position.y, position.z);
        v2XCubeMesh.material.color.setHex(color);
        v2XCubeMesh.children[0].material.color.setHex(color);
        v2XCubeMesh.rotation.set(0, 0, heading);
        return v2XCubeMesh;
    }

    drawCube(obstacle) {
        const group = new THREE.Group();
        const { length, width, height, positionX, positionY, type, heading, confidence = 0.5, id } = obstacle;
        const color = this.colors.obstacleColorMapping[type] || this.colors.obstacleColorMapping.DEFAULT;
        const position = this.coordinates.applyOffset({
            x: positionX,
            y: positionY,
        });
        if (confidence > 0) {
            const geometry = new THREE.BoxGeometry(length, width, confidence < 1 ? height * confidence : height);
            const material = new THREE.MeshBasicMaterial({ color });
            const solidBox = new THREE.BoxHelper(new THREE.Mesh(geometry, material));
            solidBox.material.color.set(color);
            solidBox.position.z =
                confidence < 1 ? ((height || DEFAULT_HEIGHT) / 2) * confidence : (height || DEFAULT_HEIGHT) / 2;
            group.add(solidBox);
        }
        if (confidence < 1) {
            const dashBox = drawDashedBox(length, width, height * (1 - confidence), color);
            dashBox.position.z = ((height || DEFAULT_HEIGHT) / 2) * (1 - confidence);
            group.add(dashBox);
        }
        group.position.set(position.x, position.y, 0);
        group.rotation.set(0, 0, heading);
        return group;
    }

    drawTexts(obstacle, autoDrivingCar) {
        const { positionX, positionY, height, id, source } = obstacle;
        const { obstacleDistanceAndSpeed, obstacleId, obstaclePriority, obstacleInteractiveTag, v2x } =
            this.option.layerOption.Perception;
        const isBirdView = this.view.viewType === 'Overhead' || this.view.viewType === 'Map';
        const isV2x = source === 'v2x';
        const textMeshs = [];
        const { positionX: adcX, positionY: adcY, heading: adcHeading } = autoDrivingCar ?? {};
        const adcPosition = new THREE.Vector3(adcX, adcY, 0);
        const obstaclePosition = new THREE.Vector3(positionX, positionY, (height || DEFAULT_HEIGHT) / 2);
        const initPosition = this.coordinates.applyOffset({
            x: positionX,
            y: positionY,
            z: height || DEFAULT_HEIGHT,
        });
        const lineSpacing = 0.5;
        const deltaX = isBirdView ? 0.0 : lineSpacing * Math.cos(adcHeading);
        const deltaY = isBirdView ? 0.7 : lineSpacing * Math.sin(adcHeading);
        const deltaZ = isBirdView ? 0.0 : lineSpacing;
        let lineCount = 0;

        if (obstacleDistanceAndSpeed) {
            const distance = adcPosition.distanceTo(obstaclePosition).toFixed(1);
            const speed = obstacle.speed.toFixed(1);
            const speedAndDistanceText = this.text.drawText(
                `(${distance}m,${speed}m/s)`,
                this.colors.textColor,
                initPosition,
            );
            if (speedAndDistanceText) {
                textMeshs.push(speedAndDistanceText);
                lineCount += 1;
            }
        }

        if (obstacleId) {
            const idPosition = {
                x: initPosition.x + lineCount * deltaX,
                y: initPosition.y + lineCount * deltaY,
                z: initPosition.z + lineCount * deltaZ,
            };
            const idText = this.text.drawText(id, this.colors.textColor, idPosition);
            if (idText) {
                textMeshs.push(idText);
                lineCount += 1;
            }
        }

        if (obstaclePriority) {
            const priority = obstacle.obstaclePriority?.priority;
            if (priority && priority !== 'NORMAL') {
                const priorityPosition = {
                    x: initPosition.x + lineCount * deltaX,
                    y: initPosition.y + lineCount * deltaY,
                    z: initPosition.z + lineCount * deltaZ,
                };
                const priorityText = this.text.drawText(priority, this.colors.textColor, priorityPosition);
                if (priorityText) {
                    textMeshs.push(priorityText);
                    lineCount += 1;
                }
            }
        }

        if (obstacleInteractiveTag) {
            const interactiveTag = obstacle.interactiveTag?.interactiveTag;
            if (interactiveTag && interactiveTag !== 'NONINTERACTION') {
                const interactiveTagPosition = {
                    x: initPosition.x + lineCount * deltaX,
                    y: initPosition.y + lineCount * deltaY,
                    z: initPosition.z + lineCount * deltaZ,
                };
                const interactiveTagText = this.text.drawText(
                    interactiveTag,
                    this.colors.textColor,
                    interactiveTagPosition,
                );
                if (interactiveTagText) {
                    textMeshs.push(interactiveTagText);
                    lineCount += 1;
                }
            }
        }

        if (isV2x && v2x) {
            const v2xType = obstacle.v2xInfo?.v2xType;
            if (v2xType) {
                v2xType.forEach((t) => {
                    const v2xTypePosition = {
                        x: initPosition.x + lineCount * deltaX,
                        y: initPosition.y + lineCount * deltaY,
                        z: initPosition.z + lineCount * deltaZ,
                    };
                    const v2xTypeText = this.text.drawText(t, this.colors.textColor, v2xTypePosition);
                    if (v2xTypeText) {
                        textMeshs.push(v2xTypeText);
                        lineCount += 1;
                    }
                });
            }
        }
        return textMeshs;
    }
}
