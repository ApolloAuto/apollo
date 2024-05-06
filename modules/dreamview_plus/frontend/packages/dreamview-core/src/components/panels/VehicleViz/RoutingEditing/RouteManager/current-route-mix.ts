import { CurrentRouteLoop, CurrentRouteMixs } from './type';

export class CurrentRouteMix {
    private currentRouteLoop: CurrentRouteLoop;

    constructor(initRouteMixs: CurrentRouteMixs) {
        this.currentRouteLoop = initRouteMixs.currentRouteLoop;
    }

    getCurrentRouteMix() {
        return { currentRouteLoop: this.currentRouteLoop };
    }

    setCurrentRouteMix(currentRouteMix: CurrentRouteMixs) {
        if (currentRouteMix.currentRouteLoop) {
            this.currentRouteLoop = currentRouteMix.currentRouteLoop;
        }
    }
}
