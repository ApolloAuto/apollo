import { PayloadAction } from '@dreamview/dreamview-core/src/store/base/Reducer';

export enum ACTIONS {
    UPDATE_MENU = 'UPDATE_MENU',
    UPDATA_CERT_STATUS = 'UPDATA_CERT_STATUS',
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

export type UpdateMenuPayload = ENUM_MENU_KEY;

export type UpdateMenuAction = PayloadAction<ACTIONS.UPDATE_MENU, UpdateMenuPayload>;

export type ChangeCertStatusPayload = ENUM_CERT_STATUS;

export type ChangeCertStatusAction = PayloadAction<ACTIONS.UPDATA_CERT_STATUS, ENUM_CERT_STATUS>;
