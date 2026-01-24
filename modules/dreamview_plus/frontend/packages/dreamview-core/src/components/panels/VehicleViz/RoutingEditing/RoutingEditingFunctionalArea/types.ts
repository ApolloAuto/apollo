export enum FunctionalNameEnum {
    RELOCATE = 'relocate',
    WAYPOINT = 'waypoint',
    LOOP = 'loop',
    FAVORITE = 'favorite',
    // 室内定位
    INDOOR_LOCALIZATION = 'indoor_localization',
    RULE = 'Rule',
    COPY = 'Copy',
}

// 互斥工具名称
export enum MutexToolNameEnum {
    RELOCATE = 'relocate',
    WAYPOINT = 'waypoint',
    LOOP = 'loop',
    RULE = 'Rule',
    COPY = 'Copy',
    // 室内定位
    INDOOR_LOCALIZATION = 'indoor_localization',
}

export enum CheckCycleRouting {
    CYCLE = 0,
    NO_CYCLE = 1,
}

export enum CommonRoutingOrigin {
    FROM_NOT_FULLSCREEN = 'NOT_FULLSCREEN',
    FROM_FULLSCREEN = 'FULLSCREEN',
}
