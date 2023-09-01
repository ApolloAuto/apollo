import { PayloadAction } from '../base/Reducer';
import { INIT_USER_INFO } from './actionTypes';

export type IInitState = {
    userInfo: {
        avatar_url: string;
        displayname: string;
        id: string;
    };
    isLogin: boolean;
};

type InitUserInfoAction = PayloadAction<typeof INIT_USER_INFO, IInitState>;

export const initUserInfo = (payload: IInitState): InitUserInfoAction => ({
    type: INIT_USER_INFO,
    payload,
});

export type CombineAction = InitUserInfoAction;
