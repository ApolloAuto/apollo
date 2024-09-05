import { noop } from 'lodash';
import { message } from '@dreamview/dreamview-ui';
import { PayloadAction } from '../base/Reducer';
import { INIT_USER_INFO, ChangeAccountAction, ACTIONS } from './actionTypes';
import { PluginApi, OtherApi } from '../../services/api';
import { AsyncAction } from '../base/Middleware';

function messageError(err: any) {
    const code = err.data.info.code;
    if (code === 50008 || code === 35004) {
        return;
    }
    message({
        type: 'error',
        content: err.data.info.message,
    });
}

export type IInitState = {
    userInfo: {
        avatar_url: string;
        displayname: string;
        id: string;
    };
    isLogin: boolean;
    account: any;
};

type InitUserInfoAction = PayloadAction<typeof INIT_USER_INFO, Omit<IInitState, 'account'>>;

export const initUserInfo = (payload: IInitState): InitUserInfoAction => ({
    type: INIT_USER_INFO,
    payload,
});

export const updateSubscribe = (pluginApi: PluginApi): AsyncAction<any, ChangeAccountAction> => {
    noop();
    return async (dispatch) => {
        try {
            const account = await pluginApi.getSubscribeAccountInfo();
            dispatch({
                type: ACTIONS.CHANGE_ACCOUNT,
                payload: account,
            });
        } catch (err) {
            messageError(err);
        }
    };
};

export type CombineAction = InitUserInfoAction | AsyncAction<any, ChangeAccountAction>;
