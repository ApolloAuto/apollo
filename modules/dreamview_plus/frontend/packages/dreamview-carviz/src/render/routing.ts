import { disposeMesh, drawThickBandFromPoints } from '../utils/common';
import { zOffset } from '../constant/common';

export default class Routing {
    private routePathMeshs;

    private scene;

    private lastRoutingTime;

    private option;

    private coordinates;

    constructor(scene, option, coordinates) {
        this.scene = scene;
        this.option = option;
        this.routePathMeshs = [];
        this.lastRoutingTime = -1;
        this.coordinates = coordinates;
    }

    update(routingTime, routePath) {
        if (!this.option.layerOption.Routing.routingLine) {
            this.dispose();
            return;
        }
        if (this.lastRoutingTime === routingTime || !routePath || routePath.length === 0) {
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        this.lastRoutingTime = routingTime;
        this.dispose();

        routePath.forEach((path) => {
            let points = path.point || [];
            if (points.length !== 0) {
                points = this.coordinates.applyOffsetToArray(points);
                const pathMesh = drawThickBandFromPoints(points, {
                    color: 0xff0000,
                    opacity: 0.6,
                    lineWidth: 0.2,
                });
                pathMesh.renderOrder = 0;
                if (pathMesh) {
                    pathMesh.position.z = zOffset.routing;
                    this.scene.add(pathMesh);
                    this.routePathMeshs.push(pathMesh);
                }
            }
        });
    }

    dispose() {
        if (this.routePathMeshs.length) {
            this.routePathMeshs.forEach((path) => {
                disposeMesh(path);
                this.scene.remove(path);
            });
            this.routePathMeshs = [];
        }
    }
}
