import { apollo } from '@dreamview/dreamview';

/**
 * 主要API类型枚举
 */
export enum MainApiTypes {
    StartRecordPackets = 'StartDataRecorder',
    GetInitData = 'GetInitData',
    StopRecordPackets = 'StopDataRecorder',
    SaveRecordPackets = 'SaveDataRecorder',
    DeleteRecordPackets = 'DeleteDataRecorder',
    ResetRecordProgress = 'ResetRecordProgress',
    StartPlayRecorder = 'StartPlayRecorder',
    StartPlayRtkRecorder = 'StartPlayRtkRecorder',
    PlayRecorderAction = 'PlayRecorderAction',
    HMIAction = 'HMIAction',
    SimHMIAction = 'SimHMIAction',
    Dump = 'Dump',
    Reset = 'Reset',
    GetDataHandlerConf = 'GetDataHandlerConf',
    TriggerPncMonitor = 'TriggerPncMonitor',
    GetDefaultRoutings = 'GetDefaultRoutings',
    SendScenarioSimulationRequest = 'SendScenarioSimulationRequest',
    CheckMapCollectStatus = 'CheckMapCollectStatus',
    StartRecordMapData = 'StartRecordMapData',
    StopRecordMapData = 'StopRecordMapData',
    StartMapCreator = 'StartMapCreator',
    BreakMapCreator = 'BreakMapCreator',
    ExportMapFile = 'ExportMapFile',
    StopScenarioSimulation = 'StopScenarioSimulation',
    ResetScenarioSimulation = 'ResetScenarioSimulation',
    DeleteDefaultRouting = 'DeleteDefaultRouting',
    SaveDefaultRouting = 'SaveDefaultRouting',
    GetStartPoint = 'GetStartPoint',
    SetStartPoint = 'SetStartPoint',
    CheckCycleRouting = 'CheckCycleRouting',
    CheckRoutingPoint = 'CheckRoutingPoint',
    SendRoutingRequest = 'SendRoutingRequest',
    ResetSimControl = 'Reset',
    SendDefaultCycleRoutingRequest = 'SendDefaultCycleRoutingRequest',
    SendParkingRoutingRequest = 'SendParkingRoutingRequest',
    GetMapElementIds = 'GetMapElementIds',
    GetMapElementsByIds = 'GetMapElementsByIds',
    AddObjectStore = 'AddOrModifyObjectToDB',
    DeleteObjectStore = 'DeleteObjectToDB',
    PutObjectStore = 'AddOrModifyObjectToDB',
    GetObjectStore = 'GetObjectFromDB',
    GetTuplesObjectStore = 'GetTuplesWithTypeFromDB',
    StartTerminal = 'StartTerminal',
    RequestRoutePath = 'RequestRoutePath',
    SendIndoorLocalizationInitPointRequest = 'SendIndoorLocalizationInitPointRequest',
    GetMapStartPoint = 'GetMapStartPoint',
    CheckIndoorLocalizationInitPointStatus = 'CheckIndoorLocalizationInitPointStatus',
}

/**
 * 插件API类型枚举
 */
export enum PluginApiTypes {
    PluginRequest = 'PluginRequest',
}

/**
 * 其他API类型枚举
 */
export enum OtherApiTypes {
    SendScenarioSimulationRequest = 'SendScenarioSimulationRequest',
    StopScenarioSimulation = 'StopScenarioSimulation',
    ResetScenarioSimulation = 'ResetScenarioSimulation',
}

/**
 * 流数据名称枚举
 */
export enum StreamDataNames {
    SIM_WORLD = 'simworld',
    CAMERA = 'camera',
    HMI_STATUS = 'hmistatus',
    // 仿真专用HMI数据
    SIM_HMI_STATUS = 'simhmistatus',
    POINT_CLOUD = 'pointcloud',
    Map = 'map',
    Obstacle = 'obstacle',
    Cyber = 'cyber',
}

/**
 * 插件API名称枚举
 */
export enum PluginApiNames {
    DownloadRecord = 'DownloadRecord',
    CheckCertStatus = 'CheckCertStatus',
    GetRecordsList = 'GetRecordsList',
    GetAccountInfo = 'GetAccountInfo',
    GetVehicleInfo = 'GetVehicleInfo',
    ResetVehicleConfig = 'ResetVehicleConfig',
    RefreshVehicleConfig = 'RefreshVehicleConfig',
    UploadVehicleConfig = 'UploadVehicleConfig',
    GetV2xInfo = 'GetV2xInfo',
    RefreshV2xConf = 'RefreshV2xConf',
    UploadV2xConf = 'UploadV2xConf',
    ResetV2xConfig = 'ResetV2xConf',
    GetDynamicModelList = 'GetDynamicModelList',
    DownloadDynamicModel = 'DownloadDynamicModel',
    GetScenarioSetList = 'GetScenarioSetList',
    DownloadScenarioSet = 'DownloadScenarioSet',
    DownloadHDMap = 'DownloadMap',
    GetMapList = 'GetMapList',
}

/**
 * HMI操作枚举
 */
export enum HMIActions {
    StopRecord = 'STOP_RECORD',
    StartAutoDrive = 'ENTER_AUTO_MODE',
    LOAD_DYNAMIC_MODELS = 'LOAD_DYNAMIC_MODELS',
    ChangeScenariosSet = 'CHANGE_SCENARIO_SET',
    ChangeScenarios = 'CHANGE_SCENARIO',
    ChangeMode = 'CHANGE_MODE',
    ChangeMap = 'CHANGE_MAP',
    ChangeVehicle = 'CHANGE_VEHICLE',
    ChangeDynamic = 'CHANGE_DYNAMIC_MODEL',
    LoadRecords = 'LOAD_RECORDS',
    LoadRecord = 'LOAD_RECORD',
    LoadScenarios = 'LOAD_SCENARIOS',
    LoadRTKRecords = 'LOAD_RTK_RECORDS',
    LoadMaps = 'LOAD_MAPS',
    ChangeRecord = 'CHANGE_RECORD',
    ChangeRTKRecord = 'CHANGE_RTK_RECORD',
    DeleteRecord = 'DELETE_RECORD',
    DeleteHDMap = 'DELETE_MAP',
    DeleteVehicle = 'DELETE_VEHICLE_CONF',
    DeleteV2X = 'DELETE_V2X_CONF',
    DeleteScenarios = 'DELETE_SCENARIO_SET',
    DeleteDynamic = 'DELETE_DYNAMIC_MODEL',
    ChangeOperation = 'CHANGE_OPERATION',
    StartModule = 'START_MODULE',
    StopModule = 'STOP_MODULE',
    SetupMode = 'SETUP_MODE',
    ResetMode = 'RESET_MODE',
    DISENGAGE = 'DISENGAGE',
}

/**
 * SIM操作枚举
 */
export enum SimHMIAction {
    LOAD_SCENARIOS = 'LOAD_SCENARIOS',
    CHANGE_SCENARIO = 'CHANGE_SCENARIO',
}

/**
 * HMI数据负载类型
 */
export type HMIDataPayload = {
    action: HMIActions | SimHMIAction;
    value?: string;
};

export enum ENUM_DOWNLOAD_STATUS {
    DOWNLOADED = 'downloaded',
    Fail = 'FAIL',
    NOTDOWNLOAD = 'notDownloaded',
    DOWNLOADING = 'downloading',
    TOBEUPDATE = 'toBeUpdated',
}

export type AccountInfo = {
    avatar_url: string;
    displayname: string;
    id: string;
    map_prerogative: boolean;
};

export type VehicleInfoRecord = {
    [id: string]: {
        vehicle_id: typeof id;
        // 车辆编号（车辆名称）
        vin: string;
        // 车辆类型（type）
        vtype: string;
        status: ENUM_DOWNLOAD_STATUS;
        percentage: number;
    };
};

export type VehicleInfo = {
    data_type: string;
    vehicle_id: string;
};

export type V2xInfoRecord = {
    [id: string]: {
        status: ENUM_DOWNLOAD_STATUS;
        // v2x名称
        obu_in: string;
        percentage: number;
    };
};

export type V2xInfo = {
    v2x_id: number;
};

export type DynamicModelRecord = {
    [name: string]: {
        name: string;
        status: ENUM_DOWNLOAD_STATUS;
        percentage: number;
    };
};

export type DynamicModel = {
    dynamic_model_name: string;
    status: ENUM_DOWNLOAD_STATUS;
    percentage: number;
    resource_type: 'dynamic_model';
    resource_id: string;
};

export type ScenarioSet = {
    scenario_set_id: string;
    status: ENUM_DOWNLOAD_STATUS;
    scenarios: {
        [scenariosId: string]: {
            name: string;
            status: ENUM_DOWNLOAD_STATUS;
        };
    };
    public: boolean;
    category: string;
    percentage: number;
    resource_id: string;
    resource_type: 'scenario';
};

export type ScenarioSetRecord = Record<string, ScenarioSet>;

export type HDMapSet = {
    percentage: number;
    resource_id: string;
    resource_type: string;

    data_type: string;
    status: ENUM_DOWNLOAD_STATUS;
};

export type DefaultPoint = {
    x: number;
    y: number;
    z?: number;
    heading?: number;
};

export type DefaultRouting = {
    name: string;
    cycleNumber?: number;
    point: DefaultPoint[];
};

export type DefaultRoutings = {
    defaultRoutings: DefaultRouting[];
};
export type StartScenario = {
    fromScenario: boolean;
    end?: {
        x: number;
        y: number;
        z?: number;
        heading?: number;
    };
    waypoint?: any;
};

export type StartCycle = {
    end: {
        x: number;
        y: number;
        z?: number;
        heading?: number;
    };
    waypoint: any;
    cycleNumber: number;
};

export type SaveDefaultRoutingInfo = {
    name: string;
    point: DefaultPoint[];
    routingType: string;
    cycleNumber?: number;
};

export enum RoutingType {
    DEFAULT_ROUTING = 'defaultRouting',
}

export type GetStartPointInfo = DefaultPoint;

export type SetStartPointInfo = {
    point: DefaultPoint;
};

export type CheckCycleRoutingInfo = {
    start: DefaultPoint;
    end: DefaultPoint;
};

export type IsCheckCycleRouting = {
    isCycle: boolean;
};

export type CheckRoutingPointInfo = {
    point: {
        id: number;
        x: number;
        y: number;
        z?: number;
        heading?: number;
    };
};

export type CheckRoutingPointLegal = {
    isLegal: number;
};

export type SendRoutingRequestInfo = {
    end: DefaultPoint;
    waypoint: DefaultPoint[];
};

export type SendDefaultCycleRoutingRequestInfo = {
    end: DefaultPoint;
    waypoint: DefaultPoint[];
    cycleNumber: number;
};

export type SendParkingRoutingRequestInfo = any;

export type GetMapElementIdsInfo = {
    point?: {
        x: number;
        y: number;
    };
    radius?: number;
};

type IMapElementIds = apollo.dreamview.IMapElementIds;

export type GetMapElementsByIdsInfo = {
    param: {
        mapElementIds: IMapElementIds;
    };
};

export enum CHECK_MAP_COLLECT_STATUS {
    OK = 'Ok',
    LOADING = 'Loading',
    WARNING = 'Warning',
    ERROR = 'Error',
}

export type CheckMapCollectInfo = {
    Gps: {
        info: string;
        status: CHECK_MAP_COLLECT_STATUS;
    };
    Lidar: {
        info: string;
        status: CHECK_MAP_COLLECT_STATUS;
    };
    Localization: {
        info: string;
        status: CHECK_MAP_COLLECT_STATUS;
    };
    Lidar2world: {
        info: string;
        status: CHECK_MAP_COLLECT_STATUS;
    };
};

export enum CREATE_MAP_FILE_STATUS {
    OK = 'Ok',
    CREATING = 'Creating',
    ERROR = 'Error',
}

export type CreateMapFileInfo = {
    progress: number;
    mapFilePath: string;
    mapCreatorTime?: string;
    status: CREATE_MAP_FILE_STATUS;
};

export type ExportMapFileInfo = {
    map_file_url: string;
};

export type HMIActionsOperationInfo = {
    isOk: boolean;
};

export type HDMapSwitchInfo = {
    isOk: boolean;
};

export enum OBJECT_STORE_TYPE {
    CHART = 'chart',
}
export interface IAddObjectStoreParams {
    key: string;
    value: any;
}

export interface IPutObjectStoreParams {
    key: string;
    value: any;
}

export interface IDeleteObjectStoreParams {
    key: string;
}

export interface IGetObjectStoreParams {
    key: string;
}

export interface IGetTuplesObjectStoreParams {
    type: OBJECT_STORE_TYPE;
}

export interface RoutePathInfoPoint {
    x: number;
    y: number;
    z: number;
}

export interface RoutePathInfoItem {
    point: RoutePathInfoPoint[];
}
export interface RoutePathInfo {
    routingTime: number;
    routePath: RoutePathInfoItem[];
}
