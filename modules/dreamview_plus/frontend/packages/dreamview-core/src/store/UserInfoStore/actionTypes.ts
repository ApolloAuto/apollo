import { PayloadAction } from '@dreamview/dreamview-core/src/store/base/Reducer';

export enum ACTIONS {
    INIT_USER_INFO = 'INIT_USER_INFO',
    CHANGE_ACCOUNT = 'CHANGE_ACCOUNT',
}

export const INIT_USER_INFO = 'INIT_USER_INFO';

export type ChangeAccountAction = PayloadAction<ACTIONS.CHANGE_ACCOUNT>;
