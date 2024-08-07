import { PayloadAction } from '@dreamview/dreamview-core/src/store/base/Reducer';
import { apollo } from '@dreamview/dreamview';

export const enum ACTIONS {
    TOGGLE_MODULE = 'TOGGLE_MODULE',
    TOGGLE_CODRIVER_FLAG = 'TOGGLE_CODRIVER_FLAG',
    TOGGLE_MUTE_FLAG = 'TOGGLE_MUTE_FLAG',
    UPDATE_STATUS = 'UPDATE_STATUS',
    UPDATE = 'UPDATE',
    UPDATE_VEHICLE_PARAM = 'UPDATE_VEHICLE_PARAM',
    UPDATE_DATA_COLLECTION_PROGRESS = 'UPDATE_DATA_COLLECTION_PROGRESS',
    UPDATE_PREPROCESS_PROGRESS = 'UPDATE_PREPROCESS_PROGRESS',
    CHANGE_TRANSLATION = 'CHANGE_TRANSLATION',
    CHANGE_INTRINSIC = 'CHANGE_INTRINSIC',
    CHANGE_MODE = 'CHANGE_MODE',
    CHANGE_OPERATE = 'CHANGE_OPERATE',
    CHANGE_RECORDER = 'CHANGE_RECORDER',
    CHANGE_RTK_RECORDER = 'CHANGE_RTK_RECORDER',
    CHANGE_DYNAMIC = 'CHANGE_DYNAMIC',
    CHANGE_SCENARIOS = 'CHANGE_SCENARIOS',
    CHANGE_MAP = 'CHANGE_MAP',
    CHANGE_VEHICLE = 'CHANGE_VEHICLE',
}

export enum SIM_CONTROL_STATUS {
    OK = 'OK',
    UNKNOWN = 'UNKNOWN',
}

export interface IPoint2D {
    x?: number;
    y?: number;
}

export interface IScenarioInfo {
    scenarioId?: string;
    scenarioName?: string;
    mapName?: string;
    startPoint?: IPoint2D;
}
export interface IScenarioSet {
    scenarioSetName?: string;
    scenarios?: IScenarioInfo[];
}
export interface IformatScenarioSet {
    scenarioSetId: string;
    name: string;
    scenarios: {
        mapName: string;
        scenarioId: string;
        scenarioName: string;
    }[];
}

// 数据包下载状态
export enum RECORDER_DOWNLOAD_STATUS {
    // 下载中
    DOWNLOADING,
    // 下载完成
    DOWNLOADEND,
}

// 数据包预加载
export enum RECORDER_LOAD_STATUS {
    // 下载中
    NOT_LOAD = 'NOT_LOAD',
    LOADING = 'LOADING',
    LOADED = 'LOADED',
}

export interface IRecordSInfo {
    loadRecordStatus: RECORDER_LOAD_STATUS;
    totalTimeS: number;
    recordFilePath: string;
}

// 车辆类型
export enum VEHICLE_TYPE {
    '未设置',
    'DKIT_LITE',
    'DKIT_STANDARD',
    'DKIT_ADVANCED_NE_S',
    'DKIT_ADVANCED_SNE_R',
    'DKIT_LITE_S',
    'DKIT_STANDARD_S',
    'DKIT_CHALLENGE',
}

export enum ENUM_DATARECORD_PROCESS_STATUS {
    FATAL = 'FATAL',
    OK = 'OK',
}

export enum ENUM_RTKRECORD_PROCESS_STATUS {
    FATAL = 'FATAL',
    OK = 'OK',
}

export enum ENUM_DATARECORD_RESOURCE_STATUS {
    OK = 'UNKNOWN',
    ERROR = 'ERROR',
    WARN = 'WARN',
}

export enum COMPONENT_STATUS {
    FATAL = 'FATAL',
    OK = 'OK',
}

export interface COMPONENTS {
    message: string;
    status: COMPONENT_STATUS;
}

export enum CURRENT_MODE {
    NONE = 'none',
    DEFAULT = 'Default',
    PERCEPTION = 'Perception',
    PNC = 'Pnc',
    VEHICLE_TEST = 'Vehicle Test',
    MAP_COLLECT = 'Map Collect',
    MAP_EDITOR = 'Map Editor',
    CAMERA_CALIBRATION = 'Camera Calibration',
    LiDAR_CALIBRATION = 'Lidar Calibration',
    DYNAMICS_CALIBRATION = 'Dynamics Calibration',
    CANBUS_DEBUG = 'Canbus Debug',
}

export enum PREPROCESS_STATUS {
    SUCCESS = 'SUCCESS',
    FAIL = 'FAIL',
    UNKNOWN = 'UNKNOWN',
}

export enum HMIModeOperation {
    None = 'None',
    PLAY_RECORDER = 'Record',
    // SIM_DEBUG = 'SIM_DEBUG',
    SIM_CONTROL = 'Sim_Control',
    SCENARIO = 'Scenario_Sim',
    AUTO_DRIVE = 'Auto_Drive',
    WAYPOINT_FOLLOW = 'Waypoint_Follow',
}

export interface SIM_WORLD_STATUS {
    currentCameraSensorChannel: string;
    currentMap: string;
    currentMode: CURRENT_MODE;
    currentRecordStatus: any;
    currentRecordId: string;
    currentScenarioId: string;
    currentScenarioSetId: string;
    currentVehicle: string;
    operations: HMIModeOperation[];
    currentOperation: HMIModeOperation;
    currentVehicleType: VEHICLE_TYPE;
    dockerImage: string;
    dynamicModels: string[];
    maps: string[];
    modes: string[];
    modules: Record<string, boolean>;
    modulesLock: Record<string, boolean>;
    monitoredComponents: Record<string, COMPONENTS>;
    passengerMsg: string;
    records: Record<string, IRecordSInfo>;
    scenarioSet: Record<string, IScenarioSet>;
    vehicles: string[];
    backendShutdown: boolean;
    globalComponents: {
        DataRecorder: {
            processStatus: {
                status: ENUM_DATARECORD_PROCESS_STATUS;
            };
            resourceStatus: {
                status: ENUM_DATARECORD_RESOURCE_STATUS;
            };
        };
    };
    otherComponents: Record<
        string,
        {
            message: string;
            status: SIM_CONTROL_STATUS;
        }
    >;
}

export type IInitState = {
    prevStatus: any;
    modes: string[];
    currentMode: CURRENT_MODE;
    dynamicModels: string[];
    vehicles: string[];
    currentVehicle: string;
    dockerImage: string;
    currentDynamicModel: string;
    otherComponents: Record<
        string,
        {
            message: string;
            status: SIM_CONTROL_STATUS;
        }
    >;
    simOtherComponents: Record<
        string,
        {
            message: string;
            status: SIM_CONTROL_STATUS;
        }
    >;

    maps: string[];
    currentMap: string;
    modules: Map<string, boolean>;
    modulesLock: Map<string, boolean>;

    records: Record<string, IRecordSInfo>;
    rtkRecords: string[];
    currentRecordId: string;
    currentRtkRecordId: string;
    currentScenarioSetId: string;
    currentScenarioId: string;
    currentScenarioName: string;
    scenarioSet: SIM_WORLD_STATUS['scenarioSet'];
    currentRecordStatus: apollo.dreamview.IRecordStatus;
    operations: apollo.dreamview.HMIModeOperation[];
    currentOperation: HMIModeOperation;

    globalComponents: {
        DataRecorder: {
            processStatus: {
                message: string;
                status: ENUM_DATARECORD_PROCESS_STATUS;
            };
            resourceStatus: {
                message: string;
                status: ENUM_DATARECORD_RESOURCE_STATUS;
            };
        };
        RTKPlayer?: {
            processStatus?: {
                message: string;
                status: ENUM_RTKRECORD_PROCESS_STATUS;
            };
            summary?: {
                message: string;
                status: ENUM_RTKRECORD_PROCESS_STATUS;
            };
        };
        RTKRecorder?: {
            processStatus?: {
                message: string;
                status: ENUM_RTKRECORD_PROCESS_STATUS;
            };
            summary?: {
                message: string;
                status: ENUM_RTKRECORD_PROCESS_STATUS;
            };
        };
    };
    envResourcesHDMapDisable: boolean;
    backendShutdown: boolean;

    Terminal?: {
        processStatus: {
            message: string;
            status: ENUM_DATARECORD_PROCESS_STATUS;
        };
        summary: {
            message: string;
            status: ENUM_DATARECORD_PROCESS_STATUS;
        };
    };
};

export type UpdateStatusAction = PayloadAction<ACTIONS.UPDATE_STATUS, SIM_WORLD_STATUS>;

export interface ToggleModulePayload {
    key: string;
}

export type ToggleModuleAction = PayloadAction<ACTIONS.TOGGLE_MODULE, ToggleModulePayload>;

export type ChangeModePayload = CURRENT_MODE;

export type ChangeModeAction = PayloadAction<ACTIONS.CHANGE_MODE, ChangeModePayload>;

export type ChangeOperatePayload = string;

export type ChangeOperateAction = PayloadAction<ACTIONS.CHANGE_OPERATE, ChangeOperatePayload>;

export type ChangeRecorderPayload = string;

export type ChangeRTKRecorderPayload = string;

export type ChangeDynamicPayload = string;

export type ChangeRecorderAction = PayloadAction<ACTIONS.CHANGE_RECORDER, ChangeRecorderPayload>;

export type ChangeRTKRecorderAction = PayloadAction<ACTIONS.CHANGE_RTK_RECORDER, ChangeRTKRecorderPayload>;

export type ChangeDynamicAction = PayloadAction<ACTIONS.CHANGE_RECORDER, ChangeDynamicPayload>;

export interface ChangeScenariosPayload {
    scenariosSetId: string;
    scenarioId: string;
}

export type ChangeScenariosAction = PayloadAction<ACTIONS.CHANGE_SCENARIOS, ChangeScenariosPayload>;

export type ChangeMapPayload = {
    mapSetId: string;
    mapDisableState: boolean;
};

export type ChangeMapAction = PayloadAction<ACTIONS.CHANGE_MAP, ChangeMapPayload>;

export type ChangeVehiclePayload = string;

export type ChangeVehicleAction = PayloadAction<ACTIONS.CHANGE_VEHICLE, ChangeVehiclePayload>;
