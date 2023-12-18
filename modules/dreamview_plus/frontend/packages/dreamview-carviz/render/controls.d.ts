export default class Controls {
    private minDistance;
    private maxDistance;
    private minPolarAngle;
    private maxPolarAngle;
    private enableRotate;
    private scene;
    private renderer;
    private camera;
    private enabled;
    private orbitControls;
    constructor(scene: any, renderer: any, camera: any);
    setMinDistance(distance: any): void;
    setMaxDistance(distance: any): void;
    setMinPolarAngle(angle: any): void;
    setMaxPolarAngle(angle: any): void;
    setEnableRotate(bool: any): void;
    setEnabled(bool: any): void;
    update(): void;
}
