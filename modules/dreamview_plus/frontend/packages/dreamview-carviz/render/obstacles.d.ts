import * as THREE from 'three';

export default class Obstacles {
    private obstacleMeshs;

    private speedHeadingArrows;

    private obstacleHeadingArrows;

    private textMeshs;

    private iconMeshs;

    private iconMeshTemplate;

    private solidFaceMeshTemplate;

    private solidLineMeshTemplate;

    private dashedLineMeshTemplate;

    private scene;

    private view;

    private text;

    private solidFaceCubeMeshTemplate;

    private solidLineCubeMeshTemplate;

    private dashLineCubeMeshTemplate;

    constructor(scene: any, view: any, text: any);

    update(obstacles: any, autoDrivingCar: any): void;

    reset(): void;

    drawIconMeshTemplate(): void;

    drawFaceMeshTemplate(): void;

    drawCubeTemplate(): void;

    drawObstaclePolygon(obstacle: any): THREE.Group;

    drawObstacle(obstacle: any): any;

    drawV2xCube(obstacle: any): any;

    drawCube(obstacle: any): void;

    drawTexts(obstacle: any, autoDrivingCar: any): any[];
}
