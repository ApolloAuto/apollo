import { BehaviorSubject, Subject } from 'rxjs';
import { CurrentRoute, CurrentCheckPoint } from './type';

export class CurrentRouteManager {
    private currentRouteSubject: BehaviorSubject<CurrentRoute>;

    private currentCheckPointSubject: Subject<CurrentCheckPoint>;

    constructor(initRouteValue: CurrentRoute) {
        this.currentRouteSubject = new BehaviorSubject<CurrentRoute>(initRouteValue);
        this.currentCheckPointSubject = new Subject<CurrentCheckPoint>();
    }

    getCurrentRoute(): CurrentRoute {
        return this.currentRouteSubject.value;
    }

    setCurrentRoute(setRouteValue: CurrentRoute) {
        this.currentRouteSubject.next(setRouteValue);
    }

    subScribeCurrentRoute(currentRouteObserver: (currentRouteValue: CurrentRoute) => void) {
        return this.currentRouteSubject.subscribe(currentRouteObserver);
    }

    setScribeCurrentCheckPoint(setCurrentPointValue: CurrentCheckPoint) {
        this.currentCheckPointSubject.next(setCurrentPointValue);
    }

    subScribeCurrentCheckPoint(currentCheckPointObserver: (currentPointValue: CurrentCheckPoint) => void) {
        return this.currentCheckPointSubject.subscribe(currentCheckPointObserver);
    }
}
