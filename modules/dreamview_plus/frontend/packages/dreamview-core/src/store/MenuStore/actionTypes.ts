import { PayloadAction } from '@dreamview/dreamview-core/src/store/base/Reducer';

export enum ACTIONS {
    UPDATE_MENU = 'UPDATE_MENU',
    UPDATA_CERT_STATUS = 'UPDATA_CERT_STATUS',
    UPDATE_ENVIORMENT_MANAGER = 'UPDATE_ENVIORMENT_MANAGER',
    UPDATE_ADS_MANAGER = 'UPDATE_ADS_MANAGER',
}

export enum ENUM_MENU_KEY {
    MODE_SETTING,
    ADD_PANEL,
    // SAVE_LAYOUT,
    PROFILE_MANAGEER,
    HIDDEN,
}

export enum ENUM_CERT_STATUS {
    UNKNOW,
    SUCCESS,
    FAIL,
}

export enum ENUM_ENVIORMENT_MANAGER_TAB_KEY {
    MAP = 'MAP',
    SCENARIO = 'SCENARIO',
    RECORD = 'RECORD',
}

export enum ENUM_ADS_MANAGER_TAB_KEY {
    VEHICLE = 'VEHICLE',
    V2X = 'V2X',
    DYNAMIC = 'DYNAMIC',
}

export type UpdateMenuPayload = ENUM_MENU_KEY;

export type UpdateMenuAction = PayloadAction<ACTIONS.UPDATE_MENU, UpdateMenuPayload>;

export type ChangeCertStatusPayload = ENUM_CERT_STATUS;

export type ChangeCertStatusAction = PayloadAction<ACTIONS.UPDATA_CERT_STATUS, ChangeCertStatusPayload>;

export type ChangeEnviormentResourcesPayload = ENUM_ENVIORMENT_MANAGER_TAB_KEY;

export type ChangeEnviormentResourcesAction = PayloadAction<
    ACTIONS.UPDATE_ENVIORMENT_MANAGER,
    ChangeEnviormentResourcesPayload
>;

export type ChangeAdsResourcesPayload = ENUM_ADS_MANAGER_TAB_KEY;

export type ChangeAdsResourcesAction = PayloadAction<ACTIONS.UPDATE_ADS_MANAGER, ChangeAdsResourcesPayload>;
