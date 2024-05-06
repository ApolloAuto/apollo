import { EventEmitter } from 'eventemitter3';
import { produce } from 'immer';
import set from 'lodash/set';
import * as TYPES from './actionTypes';

export interface IInitState {
    ee: EventEmitter;
    panelId?: string;
    state: any;
}

export const initState = {
    ee: new EventEmitter(),
    state: {},
};

export const reducer = (state: IInitState, action: TYPES.CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case TYPES.ACTIONS.UPDATE:
                set(draftState, ['state', action.payload.panelId, action.payload.key], action.payload.nextValue);
                break;
            default:
                break;
        }
    });
