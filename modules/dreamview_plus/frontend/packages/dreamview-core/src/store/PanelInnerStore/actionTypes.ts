import { PayloadAction } from '@dreamview/dreamview-core/src/store/base/Reducer';

export enum ACTIONS {
    UPDATE = 'UPDATE',
}

export interface UpdatePayload {
    panelId: string;
    key: string;
    nextValue: any;
}

export type UpdateAction = PayloadAction<ACTIONS.UPDATE, UpdatePayload>;

export type CombineAction = UpdateAction;
