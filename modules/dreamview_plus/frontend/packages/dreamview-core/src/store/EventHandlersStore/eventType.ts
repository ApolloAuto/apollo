export enum CustomEventTypes {
    MainConnectedEvent = 'main:connection',
    PluginConnectedEvent = 'plugin:connection',
}

export enum BusinessEventTypes {
    SimControlRoute = 'simcontrol:route',
}

export interface BusinessEventInfo {
    RoutePoint: {
        x: number;
        y: number;
        z?: number;
        heading?: number;
    };
    [BusinessEventTypes.SimControlRoute]: {
        panelId: string;
        routeInfo: {
            initialPoint: BusinessEventInfo['RoutePoint'];
            wayPoint: BusinessEventInfo['RoutePoint'][];
            cycleNumber: number;
        };
    };
}
