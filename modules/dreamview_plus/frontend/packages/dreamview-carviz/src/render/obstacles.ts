import * as THREE from 'three';
import { camelCase, memoize } from 'lodash';
import { perfMonitor } from '@dreamview/dreamview-analysis';
import { obstacleColorMapping } from '../constant/common';
import { disposeMesh, drawImge, disposeGroup, drawDashedBox, drawSolidBox } from '../utils/common';
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

    private iconObjectPool: ThreeObjectPool<THREE.Mesh>;

    private updatedTexureId: string;

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
        this.iconObjectPool = new ThreeObjectPool({
            type: POOL_TYPE.ICON,
            syncFactory: () => drawImge(iconObjectYield, 1, 1),
            dispose: (object) => (object.visible = false),
            reset: (object) => (object.visible = true),
        });
        this.updatedTexureId = null;
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

    getObstacleHeadingPath(obstacle) {
        const length = 1.5;
        const arrowLength = 0.5;
        const { height, positionX, positionY, heading } = obstacle;
        if (!positionX || !positionY) {
            return [];
        }
        const begin = new THREE.Vector3(
            this.coordinates.applyOffset({ x: positionX, y: positionY }).x,
            this.coordinates.applyOffset({ x: positionX, y: positionY }).y,
            (height || DEFAULT_HEIGHT) / 2,
        );
        const end = new THREE.Vector3(
            begin.x + length * Math.cos(heading),
            begin.y + length * Math.sin(heading),
            (height || DEFAULT_HEIGHT) / 2,
        );
        const temp = new THREE.Vector3(
            begin.x + (length - arrowLength) * Math.cos(heading),
            begin.y + (length - arrowLength) * Math.sin(heading),
            (height || DEFAULT_HEIGHT) / 2,
        );
        // 计算沿着end - temp方向，长度为0.5的箭头的两个端点
        const endToTempVec = new THREE.Vector3().subVectors(temp, end);
        const newVec1 = endToTempVec.clone().applyEuler(new THREE.Euler(0, 0, Math.PI / 6));
        const newVec2 = endToTempVec.clone().applyEuler(new THREE.Euler(0, 0, -Math.PI / 6));
        const top = new THREE.Vector3(newVec1.x + end.x, newVec1.y + end.y, (height || DEFAULT_HEIGHT) / 2);
        const bottom = new THREE.Vector3(newVec2.x + end.x, newVec2.y + end.y, (height || DEFAULT_HEIGHT) / 2);
        return [begin, end, top, end, bottom, end];
    }

    getSpeedHeadingPath(obstacle) {
        const { height, positionX, positionY, speedHeading, speed } = obstacle;
        if (!positionX || !positionY || speedHeading === undefined || !speed) {
            return [];
        }
        if (!speedHeading || speed === undefined) {
            return [];
        }
        const scale = 1 + Math.log2(speed);
        const length = 1.5 * scale;
        const arrowLength = 0.5;
        const begin = new THREE.Vector3(
            this.coordinates.applyOffset({ x: positionX, y: positionY }).x,
            this.coordinates.applyOffset({ x: positionX, y: positionY }).y,
            (height || DEFAULT_HEIGHT) / 2,
        );
        const end = new THREE.Vector3(
            begin.x + length * Math.cos(speedHeading),
            begin.y + length * Math.sin(speedHeading),
            (height || DEFAULT_HEIGHT) / 2,
        );

        const temp = new THREE.Vector3(
            begin.x + (length - arrowLength) * Math.cos(speedHeading),
            begin.y + (length - arrowLength) * Math.sin(speedHeading),
            (height || DEFAULT_HEIGHT) / 2,
        );
        // 计算沿着end - begin方向，长度为0.5的箭头的两个端点
        const endToTempVec = new THREE.Vector3().subVectors(temp, end);
        const newVec1 = endToTempVec.clone().applyEuler(new THREE.Euler(0, 0, Math.PI / 4));
        const newVec2 = endToTempVec.clone().applyEuler(new THREE.Euler(0, 0, -Math.PI / 4));
        const top = new THREE.Vector3(newVec1.x + temp.x, newVec1.y + temp.y, (height || DEFAULT_HEIGHT) / 2);
        const bottom = new THREE.Vector3(newVec2.x + temp.x, newVec2.y + temp.y, (height || DEFAULT_HEIGHT) / 2);
        return [begin, end, top, end, bottom, end];
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

    update(obstacles, sensorMeasurements, autoDrivingCar) {
        let objects = [];
        perfMonitor.mark('obstaclesUpdateStart');
        this.dispose();
        const { lidar, radar, camera } = this.option.layerOption.Perception;
        if (sensorMeasurements && (lidar || radar || camera)) {
            Object.keys(sensorMeasurements).forEach((key) => {
                const sensorType = memoizeGetSensorType(key.toLowerCase());
                if (!sensorType || !this.option.layerOption.Perception[sensorType]) {
                    return;
                }
                const measurements = sensorMeasurements[key]?.sensorMeasurement;
                if (!measurements || measurements.length === 0) {
                    return;
                }
                objects = [...objects, ...measurements];
            });
        }
        objects = [...objects, ...obstacles];
        this.updateObstacles(objects, autoDrivingCar);
        perfMonitor.mark('obstaclesUpdateEnd');
        perfMonitor.measure('obstaclesUpdate', 'obstaclesUpdateStart', 'obstaclesUpdateEnd');
    }

    updateObstacles(obstacles, autoDrivingCar) {
        if (!this.coordinates.isInitialized()) {
            return;
        }
        const { obstacleHeading, obstacleVelocity } = this.option.layerOption.Perception;
        const paths = [];
        const colors = [];
        let texts: { str: string; position: THREE.Vector3 }[] = [];
        for (let i = 0; i < obstacles.length; i += 1) {
            const obstacle = obstacles[i];
            const {
                positionX,
                positionY,
                yieldedobstacle,
                type,
                id,
                polygonPoint,
                source,
                confidence,
                height,
                length,
                width,
            } = obstacle;
            if ((!positionX && !positionY) || !this.option.layerOption.Perception[camelCase(type)]) {
                continue;
            }
            const actHeight = source === 'v2x' ? height : confidence < 1 ? confidence * height : height;
            const color = this.colors.obstacleColorMapping[type] || this.colors.obstacleColorMapping.DEFAULT;
            if (polygonPoint && polygonPoint.length !== 0) {
                const bottomPoints = this.coordinates.applyOffsetToArray(polygonPoint);
                const topPoints = bottomPoints.map((point) => ({
                    x: point.x,
                    y: point.y,
                    z: point.z + actHeight,
                }));
                for (let i = 0; i < bottomPoints.length - 1; i += 1) {
                    paths.push(bottomPoints[i], bottomPoints[i + 1]);
                    paths.push(bottomPoints[i], topPoints[i]);
                }
                paths.push(bottomPoints[bottomPoints.length - 1], bottomPoints[0]);
                paths.push(bottomPoints[bottomPoints.length - 1], topPoints[topPoints.length - 1]);

                for (let i = 0; i < topPoints.length - 1; i += 1) {
                    paths.push(topPoints[i], topPoints[i + 1]);
                }
                paths.push(topPoints[topPoints.length - 1], topPoints[0]);

                for (let i = 0; i < bottomPoints.length * 3; i += 1) {
                    const { r, g, b } = new THREE.Color(color);
                    colors.push(r, g, b, r, g, b);
                }
                if (obstacleHeading) {
                    const arrowPaths = this.getObstacleHeadingPath(obstacle);
                    if (arrowPaths.length) {
                        paths.push(...arrowPaths);
                        const { r, g, b } = new THREE.Color(0xffffff);
                        colors.push(r, g, b, r, g, b, r, g, b, r, g, b, r, g, b, r, g, b);
                    }
                }
                if (obstacleVelocity) {
                    const speedArrowPath = this.getSpeedHeadingPath(obstacle);
                    if (speedArrowPath.length) {
                        paths.push(...speedArrowPath);
                        const { r, g, b } = new THREE.Color(color);
                        colors.push(r, g, b, r, g, b, r, g, b, r, g, b, r, g, b, r, g, b);
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
            }

            let mesh = null;
            if (obstacle.subType === 'ST_TRAFFICCONE') {
                mesh = this.drawTrafficCone(obstacle);
            } else if (length && width && height && this.option.layerOption.Perception.boundingbox) {
                if (source === 'v2x') {
                    mesh = this.drawV2xCube(obstacle);
                } else {
                    mesh = this.drawCube(obstacle);
                }
            }
            if (mesh) {
                this.scene.add(mesh);
                this.obstacleMeshs.push(mesh);
            }
            const textsInfo = this.getTexts(obstacle, autoDrivingCar);
            texts = texts.concat(textsInfo);
        }
        if (paths.length && colors.length) {
            const geometry = new THREE.BufferGeometry().setFromPoints(paths);
            const material = new THREE.LineBasicMaterial({ transparent: true, vertexColors: true });
            geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(colors), 3));
            geometry.getAttribute('color').needsUpdate = true;
            geometry.getAttribute('position').needsUpdate = true;
            const obstacleLineSegments = new THREE.LineSegments(geometry, material);
            this.obstacleMeshs.push(obstacleLineSegments);
            this.scene.add(obstacleLineSegments);
        }

        if (texts.length) {
            this.updatedTexureId = `textTexture${new Date().getTime()}`;
            const { texture, canvasWidth, canvasHeight, positionItems } = this.generateTextCanvas(
                texts,
                this.colors.textColor,
            );
            texture.uuid = this.updatedTexureId;
            const lineLength = positionItems[positionItems.length - 1].lineNum + 1;
            texts.forEach((item, index) => {
                const itemTexture = texture.clone();
                itemTexture.uuid = this.updatedTexureId;
                itemTexture.offset.set(
                    positionItems[index].start / canvasWidth,
                    1 - (positionItems[index].lineNum + 1) / lineLength,
                );
                itemTexture.repeat.set(positionItems[index].width / canvasWidth, 1 / lineLength);
                const material = new THREE.SpriteMaterial({
                    map: itemTexture,
                    transparent: true,
                    depthTest: false,
                });
                material.map.colorSpace = 'srgb';
                const textMesh = new THREE.Sprite(material);
                textMesh.scale.set(positionItems[index].width / 24, canvasHeight / lineLength / 24, 1);
                textMesh.position.set(item.position.x, item.position.y, item.position.z);
                this.textMeshs.push(textMesh);
                this.scene.add(textMesh);
            });
        }
    }

    releasePool(mesh) {
        const type = mesh?.userData?.type;
        if (type === POOL_TYPE.ICON) {
            this.iconObjectPool.release(mesh);
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
        this.obstacleMeshs = [];
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

    getTexts(obstacle, autoDrivingCar) {
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
        const lineSpacing = 1;
        const deltaX = isBirdView ? 0.0 : lineSpacing * Math.cos(adcHeading);
        const deltaY = isBirdView ? 1 : lineSpacing * Math.sin(adcHeading);
        const deltaZ = isBirdView ? 0.0 : lineSpacing;
        let lineCount = 0;

        if (obstacleDistanceAndSpeed) {
            const distance = adcPosition.distanceTo(obstaclePosition).toFixed(1);
            const speed = obstacle.speed.toFixed(1);
            const speedAndDistanceText = {
                str: `(${distance}m,${speed}m/s)`,
                position: initPosition,
            };
            textMeshs.push(speedAndDistanceText);
            lineCount += 1;
        }

        if (obstacleId) {
            const idPosition = {
                x: initPosition.x + lineCount * deltaX,
                y: initPosition.y + lineCount * deltaY,
                z: initPosition.z + lineCount * deltaZ,
            };
            const idText = {
                str: id,
                position: idPosition,
            };
            textMeshs.push(idText);
            lineCount += 1;
        }

        if (obstaclePriority) {
            const priority = obstacle.obstaclePriority?.priority;
            if (priority && priority !== 'NORMAL') {
                const priorityPosition = {
                    x: initPosition.x + lineCount * deltaX,
                    y: initPosition.y + lineCount * deltaY,
                    z: initPosition.z + lineCount * deltaZ,
                };
                const priorityText = {
                    str: priority,
                    position: priorityPosition,
                };
                textMeshs.push(priorityText);
            }
            lineCount += 1;
        }

        if (obstacleInteractiveTag) {
            const interactiveTag = obstacle.interactiveTag?.interactiveTag;
            if (interactiveTag && interactiveTag !== 'NONINTERACTION') {
                const interactiveTagPosition = {
                    x: initPosition.x + lineCount * deltaX,
                    y: initPosition.y + lineCount * deltaY,
                    z: initPosition.z + lineCount * deltaZ,
                };
                const interactiveTagText = {
                    str: interactiveTag,
                    position: interactiveTagPosition,
                };
                textMeshs.push(interactiveTagText);
            }
            lineCount += 1;
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
                    const v2xTypeText = {
                        str: t,
                        position: v2xTypePosition,
                    };
                    textMeshs.push(v2xTypeText);
                });
                lineCount += 1;
            }
        }
        return textMeshs;
    }

    // 动态创建文字的canvas
    generateTextCanvas(texts: { str: string; position: THREE.Vector3 }[], color = '#fff') {
        const maxWidth = 1000;
        let canvasWidth = 0;

        const positionItems: { start: number; lineNum: number; width: number }[] = [];
        let curLine = 0;
        let curStart = 0;
        const canvas = document.createElement('canvas');
        canvas.style.background = 'rgba(255, 0, 0, 1)';
        const context = canvas.getContext('2d');
        const lineHeight = 48;
        context.font = `${lineHeight / 2}px sans-serif`;

        for (let i = 0; i < texts.length; i += 1) {
            const measureText = context.measureText(texts[i].str);
            let { width } = measureText;
            width = Math.ceil(width) + (Math.ceil(width) % 2 === 1 ? 1 : 0);

            if (width + curStart < maxWidth) {
                positionItems.push({ start: curStart, lineNum: curLine, width });
                canvasWidth = Math.max(canvasWidth, curStart + width);
                curStart += width;
            } else {
                curLine += 1;
                positionItems.push({ start: 0, lineNum: curLine, width });
                curStart = width;
                canvasWidth = Math.max(canvasWidth, width);
            }
        }
        const canvasHeight = (curLine + 1) * lineHeight;
        canvas.width = canvasWidth;
        canvas.height = canvasHeight;

        context.fillStyle = color;
        context.font = `${lineHeight / 2}px sans-serif`;
        context.textBaseline = 'middle';
        context.textAlign = 'left';
        for (let i = 0; i < positionItems.length; i += 1) {
            const { start, lineNum } = positionItems[i];
            context.fillText(texts[i].str, start, lineNum * lineHeight + lineHeight / 2);
        }
        const texture = new THREE.CanvasTexture(canvas);
        return {
            texture,
            canvasWidth,
            canvasHeight,
            positionItems,
        };
    }
}
