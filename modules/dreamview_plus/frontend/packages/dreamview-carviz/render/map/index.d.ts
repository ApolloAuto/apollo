import * as THREE from 'three';

declare class Map {
    private scene;

    mapGroup: any;

    width: any;

    height: any;

    constructor(scene: any);

    updateMap(mapData: any): void;

    getMapCenter(): THREE.Vector3;

    private getMapWidthAndHeight;
}
export default Map;
