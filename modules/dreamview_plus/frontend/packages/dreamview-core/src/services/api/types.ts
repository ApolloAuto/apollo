/**
 * 主要API类型枚举
 */
export enum MainApiTypes {
    ResetRecordProgress = 'ResetRecordProgress',
    StartPlayRecorder = 'StartPlayRecorder',
    PlayRecorderAction = 'PlayRecorderAction',
    HMIAction = 'HMIAction',
    Dump = 'Dump',
    Reset = 'Reset',
    GetDataHandlerConf = 'GetDataHandlerConf',
}

/**
 * 插件API类型枚举
 */
export enum PluginApiTypes {
    PluginRequest = 'PluginRequest',
}

/**
 * 流数据名称枚举
 */
export enum StreamDataNames {
    SIM_WORLD = 'simworld',
    CAMERA = 'camera',
    HMI_STATUS = 'hmistatus',
    POINT_CLOUD = 'pointcloud',
    Map = 'map',
    Obstacle = 'obstacle',
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
    ResetV2xConfig = 'ResetV2xConfig',
    GetDynamicModelList = 'GetDynamicModelList',
    DownloadDynamicModel = 'DownloadDynamicModel',
    GetScenarioSetList = 'GetScenarioSetList',
    DownloadScenarioSet = 'DownloadScenarioSet',
}

/**
 * HMI操作枚举
 */
export enum HMIActions {
    StopRecord = 'STOP_RECORD',
    ChangeMode = 'CHANGE_MODE',
    ChangeMap = 'CHANGE_MAP',
    ChangeVehicle = 'CHANGE_VEHICLE',
    LoadRecords = 'LOAD_RECORDS',
    ChangeRecord = 'CHANGE_RECORD',
    DeleteRecord = 'DELETE_RECORD',
    DeleteVehicle = 'DELETE_VEHICLE_CONF',
    DeleteV2X = 'DELETE_V2X_CONF',
    ChangeOperation = 'CHANGE_OPERATION',
    StartModule = 'START_MODULE',
    StopModule = 'STOP_MODULE',
    SetupMode = 'SETUP_MODE',
    ResetMode = 'RESET_MODE',
}

/**
 * HMI数据负载类型
 */
export type HMIDataPayload = {
    action: HMIActions;
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
    percentage: number;
    resource_id: string;
    resource_type: 'scenario';
};

export type ScenarioSetRecord = Record<string, ScenarioSet>;
