import * as THREE from 'three';
import { without } from 'lodash';
import { drawSegmentsFromPointsClone, drawDashedLineFromPointsClone } from '../../utils/line';
import { colorMapping, zOffset } from '../../constant/common';
import { disposeGroup, disposeMesh } from '../../utils/common';

export default class Lane {
    public laneGroupMap;

    private laneIdMeshMap;

    private scene;

    private text;

    private option;

    private drawedLaneIds;

    private currentLaneIds;

    public xmin;

    public xmax;

    public ymin;

    public ymax;

    public width;

    public height;

    public center;

    private coordinates;

    private dottedYellowLineDashedMaterialTemplate;

    private dottedWhiteLineDashedMaterialTemplate;

    private solidYellowLineBasicMaterialTemplate;

    private solidWhiteLineBasicMaterialTemplate;

    private curbLineBasicMaterialTemplate;

    private defaultLineBasicMaterialTemplate;

    private centerLineBasicMaterialTemplate;

    DOTTED_YELLOW;

    constructor(scene, text, option, coordinates) {
        this.scene = scene;
        this.option = option;
        this.coordinates = coordinates;
        this.laneGroupMap = {};
        this.laneIdMeshMap = {};
        this.text = text;
        this.drawedLaneIds = [];
        this.currentLaneIds = [];
        this.width = 0;
        this.height = 0;
        this.xmax = -Infinity;
        this.xmin = Infinity;
        this.ymin = Infinity;
        this.ymax = -Infinity;
        this.center = new THREE.Vector3(0, 0, 0);
        this.dottedYellowLineDashedMaterialTemplate = new THREE.LineDashedMaterial({
            color: colorMapping.YELLOW,
            dashSize: 3,
            gapSize: 3,
            transparent: true,
            opacity: 1,
        });
        this.dottedWhiteLineDashedMaterialTemplate = new THREE.LineDashedMaterial({
            color: colorMapping.WHITE,
            dashSize: 0.5,
            gapSize: 0.25,
            transparent: true,
            opacity: 0.4,
        });
        this.solidYellowLineBasicMaterialTemplate = new THREE.LineBasicMaterial({
            color: colorMapping.YELLOW,
            transparent: true,
            opacity: 1,
        });
        this.solidWhiteLineBasicMaterialTemplate = new THREE.LineBasicMaterial({
            color: colorMapping.WHITE,
            opacity: 1,
            transparent: true,
        });
        this.curbLineBasicMaterialTemplate = new THREE.LineBasicMaterial({
            color: colorMapping.CORAL,
            opacity: 1,
            transparent: true,
        });
        this.centerLineBasicMaterialTemplate = new THREE.LineBasicMaterial({
            color: colorMapping.GREEN,
            opacity: 1,
            transparent: true,
        });
    }

    drawLaneMesh(laneType, points) {
        let left = null;
        let right = null;
        switch (laneType) {
            case 'DOTTED_YELLOW':
                return drawDashedLineFromPointsClone(points, this.dottedYellowLineDashedMaterialTemplate);
            case 'DOTTED_WHITE':
                return drawDashedLineFromPointsClone(points, this.dottedWhiteLineDashedMaterialTemplate);
            case 'SOLID_YELLOW':
                return drawSegmentsFromPointsClone(points, this.solidYellowLineBasicMaterialTemplate);
            case 'SOLID_WHITE':
                return drawSegmentsFromPointsClone(points, this.solidWhiteLineBasicMaterialTemplate);
            case 'DOUBLE_YELLOW':
                left = drawSegmentsFromPointsClone(points, this.solidYellowLineBasicMaterialTemplate);
                right = drawSegmentsFromPointsClone(points, this.solidYellowLineBasicMaterialTemplate);
                left.add(right);
                return left;
            case 'CURB':
                return drawSegmentsFromPointsClone(points, this.curbLineBasicMaterialTemplate);
            default:
                return drawSegmentsFromPointsClone(points, this.defaultLineBasicMaterialTemplate);
        }
    }

    drawLanes(lanes) {
        if (!this.option.layerOption.Map.laneId) {
            this.disposeLaneIds();
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        this.currentLaneIds = [];
        lanes.forEach((lane) => {
            const id = lane.id.id;
            this.currentLaneIds.push(id);
            // 如果已经绘制了，则不二次绘制
            if (this.option.layerOption.Map.laneId && !this.laneIdMeshMap[id]) {
                this.drawLaneId(lane);
            }
            if (this.laneGroupMap[id]) {
                return;
            }
            const group = new THREE.Group();
            group.name = id;
            const centralLine = lane.centralCurve.segment;
            centralLine.forEach((segment) => {
                const points = this.coordinates.applyOffsetToArray(segment.lineSegment.point);
                points.forEach((p) => {
                    this.xmin = Math.min(p.x, this.xmin);
                    this.xmax = Math.max(p.x, this.xmax);
                    this.ymin = Math.min(p.y, this.ymin);
                    this.ymax = Math.max(p.y, this.ymax);
                });
                const centerLine = drawSegmentsFromPointsClone(points, this.centerLineBasicMaterialTemplate);
                centerLine.name = `CentralLine-${id}`;
                this.scene.add(centerLine);
            });
            const rightLaneType = lane.rightBoundary.boundaryType[0].types[0];
            lane.rightBoundary.curve.segment.forEach((segment) => {
                const points = this.coordinates.applyOffsetToArray(segment.lineSegment.point);
                const boundary = this.drawLaneMesh(rightLaneType, points);
                boundary.name = `RightBoundary-${id}`;
                boundary.position.z = zOffset.lane;
                group.add(boundary);
                this.scene.add(boundary);
            });

            const leftLaneType = lane.leftBoundary.boundaryType[0].types[0];
            lane.leftBoundary.curve.segment.forEach((segment) => {
                const points = this.coordinates.applyOffsetToArray(segment.lineSegment.point);
                const boundary = this.drawLaneMesh(leftLaneType, points);
                boundary.name = `LeftBoundary-${id}`;
                boundary.position.z = zOffset.lane;
                group.add(boundary);
                this.scene.add(boundary);
            });
            this.laneGroupMap[id] = group;
            this.drawedLaneIds.push(id);
        });
        this.width = this.xmax - this.xmin;
        this.height = this.ymax - this.ymin;
        this.center = new THREE.Vector3((this.xmax + this.xmin) / 2, (this.ymax + this.ymin) / 2, 0);
        this.removeOldLanes();
    }

    drawLaneId(lane) {
        const id = lane.id.id;
        if (this.laneIdMeshMap[id]) {
            return;
        }

        const centerLine = lane.centralCurve.segment;
        const position = this.coordinates.applyOffset(centerLine?.[0]?.startPosition);
        if (position) {
            position.z = 0.04;
        }

        const points = centerLine?.[0].lineSegment?.point;
        let rotationZ = 0;
        if (points && points.length >= 2) {
            const p1 = points[0];
            const p2 = points[1];
            rotationZ = Math.atan2(p2.y - p1.y, p2.x - p1.x);
        }

        const textMesh = this.text.drawText(id, colorMapping.WHITE, position);
        if (textMesh) {
            textMesh.rotation.z = rotationZ;
            this.laneIdMeshMap[id] = textMesh;
            this.scene.add(textMesh);
        }
    }

    dispose() {
        this.xmax = -Infinity;
        this.xmin = Infinity;
        this.ymax = -Infinity;
        this.ymin = Infinity;
        this.width = 0;
        this.height = 0;
        this.center = new THREE.Vector3(0, 0, 0);
        this.disposeLaneIds();
        this.disposeLanes();
    }

    disposeLanes() {
        this.drawedLaneIds = [];
        this.currentLaneIds = [];
        Object.keys(this.laneGroupMap).forEach((id) => {
            const group = this.laneGroupMap[id];
            disposeGroup(group);
            this.scene.remove(group);
        });
        this.laneGroupMap = {};
    }

    disposeLaneIds() {
        this.drawedLaneIds = [];
        this.currentLaneIds = [];
        this.text?.reset();
        Object.keys(this.laneIdMeshMap).forEach((id) => {
            const text = this.laneIdMeshMap[id];
            this.scene.remove(text);
        });
        this.laneIdMeshMap = {};
    }

    removeOldLanes() {
        const needRemovedLaneIds = without(this.drawedLaneIds, ...this.currentLaneIds);
        if (needRemovedLaneIds && needRemovedLaneIds.length) {
            needRemovedLaneIds.forEach((id) => {
                const removedGroup = this.laneGroupMap[id];
                disposeGroup(removedGroup);
                this.scene.remove(removedGroup);
                delete this.laneGroupMap[id];
                this.drawedLaneIds = [...this.currentLaneIds];

                const textMesh = this.laneIdMeshMap[id];

                if (textMesh) {
                    disposeMesh(textMesh);
                    this.scene.remove(textMesh);
                    delete this.laneIdMeshMap[id];
                }
            });
        }
    }
}
