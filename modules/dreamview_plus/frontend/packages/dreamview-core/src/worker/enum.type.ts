export enum HMIModeOperation {
    // None
    None = 0,
    // 仿真调试
    SIM_DEBUG = 1,
    // 自由仿真
    Sim_Control = 2,
    // 实车自动驾驶
    Auto_Drive = 3,
    // 循迹
    TRACE = 4,
    // 场景仿真
    Scenario_Sim = 5,
    // 播包
    Record = 6,
    // 循迹
    Waypoint_Follow = 7,
}

export enum LoadRecordStatus {
    NOT_LOAD = 1,
    LOADING = 2,
    LOADED = 3,
}

export enum PlayRecordStatus {
    // action: play continue
    RUNNING = 0,
    // action: pause
    PAUSED = 1,
    // action: default kill
    CLOSED = 2,
}

export enum ComponentStatus {
    UNKNOWN = 0,
    OK = 1,
    WARN = 2,
    ERROR = 3,
    FATAL = 4,
}

export enum DecisionType {
    IGNORE = 0, // Ignore the object
    STOP = 1, // Stop at a distance from the object
    NUDGE = 2, // Go around the object
    YIELD = 3, // Go after the object
    OVERTAKE = 4, // Go before the object
    FOLLOW = 5, // Follow the object in the same lane
    SIDEPASS = 6, // Pass the object in neighboring lane
}

export enum StopReasonCode {
    STOP_REASON_HEAD_VEHICLE = 1,
    STOP_REASON_DESTINATION = 2,
    STOP_REASON_PEDESTRIAN = 3,
    STOP_REASON_OBSTACLE = 4,
    STOP_REASON_SIGNAL = 100,
    STOP_REASON_STOP_SIGN = 101,
    STOP_REASON_YIELD_SIGN = 102,
    STOP_REASON_CLEAR_ZONE = 103,
    STOP_REASON_CROSSWALK = 104,
    STOP_REASON_EMERGENCY = 105,
    STOP_REASON_NOT_READY = 106,
    STOP_REASON_PULL_OVER = 107,
}

export enum ChangeLaneType {
    FORWARD = 0,
    LEFT = 1,
    RIGHT = 2,
}

export enum GearPosition {
    GEAR_NEUTRAL = 0,
    GEAR_DRIVE = 1,
    GEAR_REVERSE = 2,
    GEAR_PARKING = 3,
    GEAR_LOW = 4,
    GEAR_INVALID = 5,
    GEAR_NONE = 6,
}

export enum DisengageType {
    DISENGAGE_NONE = 0,
    DISENGAGE_UNKNOWN = 1,
    DISENGAGE_MANUAL = 2,
    DISENGAGE_EMERGENCY = 3,
    DISENGAGE_AUTO_STEER_ONLY = 4,
    DISENGAGE_AUTO_SPEED_ONLY = 5,
    DISENGAGE_CHASSIS_ERROR = 6,
}

export enum ObjectType {
    UNKNOWN = 0,
    UNKNOWN_MOVABLE = 1,
    UNKNOWN_UNMOVABLE = 2,
    PEDESTRIAN = 3, // pedestrian, usually determined by moving behavior.
    BICYCLE = 4, // bike, motor bike.
    VEHICLE = 5, // passenger car or truck.
    VIRTUAL = 6, // virtual object created by decision module.
    CIPV = 7, // closest in-path vehicle determined by perception module.
}

export enum SubType {
    ST_UNKNOWN = 0,
    ST_UNKNOWN_MOVABLE = 1,
    ST_UNKNOWN_UNMOVABLE = 2,
    ST_CAR = 3,
    ST_VAN = 4,
    ST_TRUCK = 5,
    ST_BUS = 6,
    ST_CYCLIST = 7,
    ST_MOTORCYCLIST = 8,
    ST_TRICYCLIST = 9,
    ST_PEDESTRIAN = 10,
    ST_TRAFFICCONE = 11,
    ST_SMALLMOT = 12,
    ST_BIGMOT = 13,
    ST_NONMOT = 14,
}

export enum Priority {
    CAUTION = 1,
    NORMAL = 2,
    IGNORE = 3,
}

export enum InteractiveTag {
    INTERACTION = 1,
    NONINTERACTION = 2,
}

export enum Source {
    HOST_VEHICLE = 0,
    V2X = 1,
}

export enum V2XType {
    NONE = 0,
    ZOMBIES_CAR = 1,
    BLIND_ZONE = 2,
}

export enum MessageSource {
    UNKNOWN = 1,
    CANBUS = 2,
    CONTROL = 3,
    DECISION = 4,
    LOCALIZATION = 5,
    PLANNING = 6,
    PREDICTION = 7,
    SIMULATOR = 8,
    HWSYS = 9,
    ROUTING = 10,
    MONITOR = 11,
    HMI = 12,
    RELATIVE_MAP = 13,
    GNSS = 14,
    CONTI_RADAR = 15,
    RACOBIT_RADAR = 16,
    ULTRASONIC_RADAR = 17,
    MOBILEYE = 18,
    DELPHI_ESR = 19,
    STORYTELLING = 20,
    TASK_MANAGER = 21,
    NANO_RADAR = 22,
}

export enum LogLevel {
    INFO = 0,
    WARN = 1,
    ERROR = 2,
    FATAL = 3,
}

export enum LaneBoundaryType {
    UNKNOWN = 0,
    DOTTED_YELLOW = 1,
    DOTTED_WHITE = 2,
    SOLID_YELLOW = 3,
    SOLID_WHITE = 4,
    DOUBLE_YELLOW = 5,
    CURB = 6,
}
