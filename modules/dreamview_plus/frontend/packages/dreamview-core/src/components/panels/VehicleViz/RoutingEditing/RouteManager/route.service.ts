import { CurrentRouteManager } from './current-route.service';
import { CurrentRouteMix } from './current-route-mix';
import { CurrentRoute, CurrentRouteMixs } from './type';

export class RouteManager {
    public currentRouteManager: CurrentRouteManager;

    public currentRouteMix: CurrentRouteMix;

    constructor(currentRoute: CurrentRoute, currentRouteMix: CurrentRouteMixs) {
        this.currentRouteManager = new CurrentRouteManager(currentRoute);
        this.currentRouteMix = new CurrentRouteMix(currentRouteMix);
    }
}
