import * as THREE from 'three';
import { colorMapping, zOffset } from '../../constant/common';
import { disposeMesh } from '../../utils/common';

export default class Lane {
    private laneWhiteDashedLine;

    private MAX_POINTS = 10000;

    private laneYellowDashedLine;

    private laneSolidLine;

    private laneYellowDashedGeometry;

    private laneWhiteDashedGeometry;

    private laneSolidGeometry;

    private laneWhiteDashMaterial;

    private laneYellowDashMaterial;

    private laneSolidMaterial;

    private laneIdMeshMap;

    private scene;

    private text;

    private option;

    private currentLaneIds;

    public xmin;

    public xmax;

    public ymin;

    public ymax;

    public width;

    public height;

    public center;

    private coordinates;

    private colors;

    constructor(scene, text, option, coordinates, colors?) {
        this.colors = colors?.colorMapping || colorMapping;
        this.scene = scene;
        this.option = option;
        this.coordinates = coordinates;
        this.laneIdMeshMap = {};
        this.text = text;
        this.currentLaneIds = [];
        this.width = 0;
        this.height = 0;
        this.xmax = -Infinity;
        this.xmin = Infinity;
        this.ymin = Infinity;
        this.ymax = -Infinity;
        this.MAX_POINTS = 10000;
        this.initLineGeometry();
        this.initLineMaterial();
    }

    getLaneLineColor(laneType) {
        switch (laneType) {
            case 'DOTTED_YELLOW':
                return new THREE.Color(this.colors.YELLOW);
            case 'DOTTED_WHITE':
                return new THREE.Color(this.colors.WHITE);
            case 'SOLID_YELLOW':
                return new THREE.Color(this.colors.YELLOW);
            case 'SOLID_WHITE':
                return new THREE.Color(this.colors.WHITE);
            default:
                return new THREE.Color(this.colors.BLACK);
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
        const whiteDashPaths: THREE.Vector3[] = [];
        const whiteDashColors: number[] = [];
        const yellowDashPaths: THREE.Vector3[] = [];
        const yellowDashColors: number[] = [];
        const solidPaths: THREE.Vector3[] = [];
        const solidColors: number[] = [];

        lanes.forEach((lane) => {
            const id = lane.id.id;
            this.currentLaneIds.push(id);
            // 如果已经绘制了，则不二次绘制
            if (this.option.layerOption.Map.laneId && !this.laneIdMeshMap[id]) {
                this.drawLaneId(lane);
            }
            const centralLine = lane.centralCurve.segment;
            const { r: centerR, g: centerG, b: centerB } = new THREE.Color(this.colors.GREEN);
            centralLine.forEach((segment) => {
                const points = this.coordinates.applyOffsetToArray(segment.lineSegment.point);
                points.forEach((p, index) => {
                    this.xmin = Math.min(p.x, this.xmin);
                    this.xmax = Math.max(p.x, this.xmax);
                    this.ymin = Math.min(p.y, this.ymin);
                    this.ymax = Math.max(p.y, this.ymax);
                    if (index !== points.length - 1) {
                        solidPaths.push(
                            new THREE.Vector3(p.x, p.y, p.z),
                            new THREE.Vector3(points[index + 1].x, points[index + 1].y, points[index + 1].z),
                        );
                        solidColors.push(centerR, centerG, centerB, centerR, centerG, centerB);
                    }
                });
            });
            const rightLaneType = lane.rightBoundary.boundaryType[0].types[0];
            const { r: rLaneLineR, g: rLaneLineG, b: rLaneLineB } = this.getLaneLineColor(rightLaneType);
            const rightIsSolid = rightLaneType.indexOf('SOLID') > -1;
            const isYellow = rightLaneType.indexOf('YELLOW') > -1;
            // @ts-nocheck
            const rightPushPaths = rightIsSolid ? solidPaths : isYellow ? yellowDashPaths : whiteDashPaths;
            const rightPushColors = rightIsSolid ? solidColors : isYellow ? yellowDashColors : whiteDashColors;
            lane.rightBoundary.curve.segment.forEach((segment) => {
                const points = this.coordinates.applyOffsetToArray(segment.lineSegment.point);
                points.forEach((p, index) => {
                    if (index !== points.length - 1) {
                        rightPushPaths.push(
                            new THREE.Vector3(p.x, p.y, p.z),
                            new THREE.Vector3(points[index + 1].x, points[index + 1].y, points[index + 1].z),
                        );
                        rightPushColors.push(rLaneLineR, rLaneLineG, rLaneLineB, rLaneLineR, rLaneLineG, rLaneLineB);
                    }
                });
            });
            const leftLaneType = lane.leftBoundary.boundaryType[0].types[0];
            const { r: lLaneLineR, g: lLaneLineG, b: lLaneLineB } = this.getLaneLineColor(leftLaneType);
            const leftIsSolid = leftLaneType.indexOf('SOLID') > -1;
            const leftPushPaths = leftIsSolid ? solidPaths : isYellow ? yellowDashPaths : whiteDashPaths;
            const leftPushColors = leftIsSolid ? solidColors : isYellow ? yellowDashColors : whiteDashColors;
            lane.leftBoundary.curve.segment.forEach((segment) => {
                const points = this.coordinates.applyOffsetToArray(segment.lineSegment.point);
                points.forEach((p, index) => {
                    if (index !== points.length - 1) {
                        leftPushPaths.push(
                            new THREE.Vector3(p.x, p.y, p.z),
                            new THREE.Vector3(points[index + 1].x, points[index + 1].y, points[index + 1].z),
                        );
                        leftPushColors.push(lLaneLineR, lLaneLineG, lLaneLineB, lLaneLineR, lLaneLineG, lLaneLineB);
                    }
                });
            });
        });

        this.laneSolidLine = this.updateLaneLineGeometry(
            this.laneSolidGeometry,
            this.laneSolidMaterial,
            this.laneSolidLine,
            solidPaths,
            solidColors,
        );

        this.laneYellowDashedLine = this.updateLaneLineGeometry(
            this.laneYellowDashedGeometry,
            this.laneYellowDashMaterial,
            this.laneYellowDashedLine,
            yellowDashPaths,
            yellowDashColors,
        );

        this.laneWhiteDashedLine = this.updateLaneLineGeometry(
            this.laneWhiteDashedGeometry,
            this.laneWhiteDashMaterial,
            this.laneWhiteDashedLine,
            whiteDashPaths,
            whiteDashColors,
        );

        this.width = this.xmax - this.xmin;
        this.height = this.ymax - this.ymin;
        this.center = new THREE.Vector3((this.xmax + this.xmin) / 2, (this.ymax + this.ymin) / 2, 0);
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

        const textMesh = this.text.drawText(id, this.colors.WHITE, position);
        if (textMesh) {
            textMesh.rotation.z = rotationZ;
            this.laneIdMeshMap[id] = textMesh;
            this.scene.add(textMesh);
        }
    }

    initLineGeometry() {
        this.laneYellowDashedGeometry = new THREE.BufferGeometry();
        this.laneYellowDashedGeometry.setAttribute(
            'position',
            new THREE.BufferAttribute(new Float32Array(this.MAX_POINTS * 3), 3),
        );
        this.laneYellowDashedGeometry.setAttribute(
            'color',
            new THREE.BufferAttribute(new Float32Array(this.MAX_POINTS * 3), 3),
        );
        this.laneWhiteDashedGeometry = new THREE.BufferGeometry();
        this.laneWhiteDashedGeometry.setAttribute(
            'position',
            new THREE.BufferAttribute(new Float32Array(this.MAX_POINTS * 3), 3),
        );
        this.laneWhiteDashedGeometry.setAttribute(
            'color',
            new THREE.BufferAttribute(new Float32Array(this.MAX_POINTS * 3), 3),
        );

        this.laneSolidGeometry = new THREE.BufferGeometry();
        this.laneSolidGeometry.setAttribute(
            'position',
            new THREE.BufferAttribute(new Float32Array(this.MAX_POINTS * 3), 3),
        );
        this.laneSolidGeometry.setAttribute(
            'color',
            new THREE.BufferAttribute(new Float32Array(this.MAX_POINTS * 3), 3),
        );
    }

    initLineMaterial() {
        this.laneSolidMaterial = new THREE.LineBasicMaterial({
            transparent: true,
            vertexColors: true,
        });
        this.laneWhiteDashMaterial = new THREE.LineDashedMaterial({
            dashSize: 0.5,
            gapSize: 0.25,
            transparent: true,
            opacity: 0.4,
            vertexColors: true,
        });
        this.laneYellowDashMaterial = new THREE.LineDashedMaterial({
            dashSize: 3,
            gapSize: 3,
            transparent: true,
            opacity: 1,
            vertexColors: true,
        });
    }

    updateLaneLineGeometry(
        geometry: THREE.BufferGeometry,
        material: THREE.LineBasicMaterial | THREE.LineDashedMaterial,
        lineSegments: THREE.LineSegments,
        updatePaths: THREE.Vector3[],
        updateColors: number[],
    ): THREE.LineSegments {
        if (!updatePaths.length || !updateColors.length) {
            return null;
        }
        if (updatePaths.length > this.MAX_POINTS) {
            this.dispose();
            this.MAX_POINTS = updatePaths.length;
            this.initLineGeometry();
            this.initLineMaterial();
        }
        const positions = geometry.attributes.position;
        const colors = geometry.attributes.color;
        updatePaths.forEach((_item, i) => {
            positions.setXYZ(i, updatePaths[i].x, updatePaths[i].y, updatePaths[i].z);
            colors.setXYZ(i, updateColors[i * 3], updateColors[i * 3 + 1], updateColors[i * 3 + 2]);
        });
        geometry.setDrawRange(0, updatePaths.length);
        geometry.getAttribute('color').needsUpdate = true;
        geometry.getAttribute('position').needsUpdate = true;
        if (!lineSegments) {
            const line = new THREE.LineSegments(geometry, material);
            lineSegments = line;
            this.scene.add(line);
        }
        lineSegments.computeLineDistances();
        lineSegments.position.z = zOffset.lane;
        return lineSegments;
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
        this.currentLaneIds = [];
        disposeMesh(this.laneSolidLine);
        disposeMesh(this.laneWhiteDashedLine);
        disposeMesh(this.laneYellowDashedLine);
        this.laneSolidLine = null;
        this.laneWhiteDashedLine = null;
        this.laneYellowDashedLine = null;
    }

    disposeLaneIds() {
        this.currentLaneIds = [];
        this.text?.reset();
        Object.keys(this.laneIdMeshMap).forEach((id) => {
            const text = this.laneIdMeshMap[id];
            this.scene.remove(text);
        });
        this.laneIdMeshMap = {};
    }
}
