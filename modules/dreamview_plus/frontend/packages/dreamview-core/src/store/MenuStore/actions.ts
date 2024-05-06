import { ACTIONS } from './actionTypes';
import * as TYPES from './actionTypes';

export const UpdateMenuAction = (payload: TYPES.ENUM_MENU_KEY): TYPES.UpdateMenuAction => ({
    type: ACTIONS.UPDATE_MENU,
    payload,
});

export const ChangeCertStatusAction = (payload: TYPES.ENUM_CERT_STATUS): TYPES.ChangeCertStatusAction => ({
    type: ACTIONS.UPDATA_CERT_STATUS,
    payload,
});

export const CloseMenuAction = (): TYPES.UpdateMenuAction => ({
    type: ACTIONS.UPDATE_MENU,
    payload: TYPES.ENUM_MENU_KEY.HIDDEN,
});

export const ChangeEnviormentResourcesAction = (
    payload: TYPES.ChangeEnviormentResourcesPayload,
): TYPES.ChangeEnviormentResourcesAction => ({
    type: ACTIONS.UPDATE_ENVIORMENT_MANAGER,
    payload,
});

export const ChangeAdsResourcesAction = (payload: TYPES.ChangeAdsResourcesPayload): TYPES.ChangeAdsResourcesAction => ({
    type: ACTIONS.UPDATE_ADS_MANAGER,
    payload,
});

export type CombineAction =
    | TYPES.UpdateMenuAction
    | TYPES.ChangeCertStatusAction
    | TYPES.ChangeEnviormentResourcesAction
    | TYPES.ChangeAdsResourcesAction;
