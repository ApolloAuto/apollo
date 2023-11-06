import { produce } from 'immer';
import * as TYPES from './actionTypes';
import { CombineAction } from './actions';

export type IInitState = {
    activeMenu: TYPES.ENUM_MENU_KEY;
    certStatus: TYPES.ENUM_CERT_STATUS;
    activeEnviormentResourceTab: TYPES.ENUM_ENVIORMENT_MANAGER_TAB_KEY;
    activeAdsResourceTab: TYPES.ENUM_ADS_MANAGER_TAB_KEY;
};

export const initState: IInitState = {
    activeMenu: TYPES.ENUM_MENU_KEY.HIDDEN,
    certStatus: TYPES.ENUM_CERT_STATUS.UNKNOW,
    activeEnviormentResourceTab: TYPES.ENUM_ENVIORMENT_MANAGER_TAB_KEY.RECORD,
    activeAdsResourceTab: TYPES.ENUM_ADS_MANAGER_TAB_KEY.VEHICLE,
};

export const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case TYPES.ACTIONS.UPDATE_MENU:
                draftState.activeMenu = action.payload;
                break;
            case TYPES.ACTIONS.UPDATA_CERT_STATUS:
                draftState.certStatus = action.payload;
                break;
            case TYPES.ACTIONS.UPDATE_ENVIORMENT_MANAGER:
                draftState.activeEnviormentResourceTab = action.payload;
                break;
            case TYPES.ACTIONS.UPDATE_ADS_MANAGER:
                draftState.activeAdsResourceTab = action.payload;
                break;
            default:
                break;
        }
    });
