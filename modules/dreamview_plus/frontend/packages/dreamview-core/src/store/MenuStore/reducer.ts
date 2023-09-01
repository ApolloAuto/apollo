import { produce } from 'immer';
import * as TYPES from './actionTypes';
import { CombineAction } from './actions';

export type IInitState = {
    activeMenu: TYPES.ENUM_MENU_KEY;
    certStatus: TYPES.ENUM_CERT_STATUS;
};

export const initState: IInitState = {
    activeMenu: TYPES.ENUM_MENU_KEY.HIDDEN,
    certStatus: TYPES.ENUM_CERT_STATUS.UNKNOW,
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
            default:
                break;
        }
    });
