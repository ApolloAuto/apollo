export enum RouteOrigin {
    EDITING_ROUTE = 'editing',
    CREATING_ROUTE = 'creating',
}

export enum PointType {
    INITIAL_POINT = 'initial_point',
    WAY_POINT = 'way_point',
    INDOOR_LOCALIZATION_POINT = 'indoor_localization_point',
}
export interface RoutePoint {
    x: number;
    y: number;
    z?: number;
    heading?: number;
}

export interface RouteInfo {
    routeOrigin: RouteOrigin;
    routePoint: {
        routeInitialPoint: RoutePoint;
        routeWayPoint: RoutePoint[];
    };
}

export type CurrentRoute = RouteInfo;

export type CurrentCheckPoint = {
    x: number;
    y: number;
    z?: number;
    type: PointType;
    heading?: number;
};

export type CurrentRouteLoop = {
    currentRouteLoopState: boolean;
    currentRouteLoopTimes?: number;
};

export type CurrentRouteMixs = {
    currentRouteLoop: CurrentRouteLoop;
};
