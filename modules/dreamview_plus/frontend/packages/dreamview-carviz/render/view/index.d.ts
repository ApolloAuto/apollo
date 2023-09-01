export default class View {
    private defaultViewDistance;
    private nearViewDistance;
    private overheadViewDistance;
    private mapViewDistance;
    private camera;
    private viewAngle;
    private controls;
    private renderer;
    viewType: string;
    constructor(camera: any, renderer: any, controls: any);
    setDefaultViewDistance(distance: any): void;
    getViewDistance(): any;
    setNearViewDistance(distance: any): void;
    getNearViewDistance(): any;
    setOverheadViewDistance(distance: any): void;
    getOverheadViewDistance(): any;
    setMapViewDistance(distance: any): void;
    getMapViewDistance(): any;
    setViewAngle(angle: any): void;
    getViewAngle(): any;
    setViewType(type: any): void;
    setView(target: any): void;
}
