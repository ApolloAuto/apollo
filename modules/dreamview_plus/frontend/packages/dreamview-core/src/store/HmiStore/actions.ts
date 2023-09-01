import Logger from '@dreamview/log';
import { ACTIONS, ChangeModeAction, ChangeOperateAction, ChangeRecorderAction } from './actionTypes';
import * as TYPES from './actionTypes';
import { MainApi } from '../../services/api';
import { AsyncAction } from '../base/Middleware';
import { noop } from '../../util/similarFunctions';

const logger = Logger.getInstance('HmiActions');

export const updateStatus = (val: TYPES.SIM_WORLD_STATUS): TYPES.UpdateStatusAction => ({
    type: ACTIONS.UPDATE_STATUS,
    payload: val,
});

export const changeMode = (
    mainApi: MainApi,
    payload: TYPES.ChangeModePayload,
    callback?: (mode: string) => void,
): AsyncAction<TYPES.IInitState, ChangeModeAction> => {
    noop();
    return async (dispatch, state) => {
        logger.debug('changeMode', { state, payload });
        await mainApi.changeSetupMode(payload);
        dispatch({
            type: ACTIONS.CHANGE_MODE,
            payload,
        });
        if (callback) {
            callback(payload);
        }
    };
};

export const changeOperate = (
    mainApi: MainApi,
    payload: TYPES.ChangeOperatePayload,
): AsyncAction<TYPES.IInitState, ChangeOperateAction> => {
    noop();
    return async (dispatch, state) => {
        logger.debug('changeOperate', { state, payload });
        await mainApi.changeOperation(payload);
        dispatch({
            type: ACTIONS.CHANGE_OPERATE,
            payload,
        });
    };
};

export const changeRecorder = (
    mainApi: MainApi,
    payload: TYPES.ChangeRecorderPayload,
): AsyncAction<TYPES.IInitState, ChangeRecorderAction> => {
    noop();
    return async (dispatch, state) => {
        logger.debug('changeRecorder', { state, payload });
        await mainApi.changeRecord(payload);
        dispatch({
            type: ACTIONS.CHANGE_RECORDER,
            payload,
        });
    };
};

export const changeScenarios = (payload: TYPES.ChangeScenariosPayload): TYPES.ChangeScenariosAction => ({
    type: ACTIONS.CHANGE_SCENARIOS,
    payload,
});

export const changeMap = (
    mainApi: MainApi,
    payload: TYPES.ChangeMapPayload,
): AsyncAction<TYPES.IInitState, TYPES.ChangeMapAction> => {
    noop();
    return async (dispatch, state) => {
        logger.debug('changeMap', { state, payload });
        await mainApi.changeMap(payload);
        dispatch({
            type: ACTIONS.CHANGE_MAP,
            payload,
        });
    };
};

export const changeVehicle = (
    mainApi: MainApi,
    payload: TYPES.ChangeVehiclePayload,
): AsyncAction<TYPES.IInitState, TYPES.ChangeVehicleAction> => {
    noop();
    return async (dispatch, state) => {
        logger.debug('changeMap', { state, payload });
        await mainApi.changeVehicle(payload);
        dispatch({
            type: ACTIONS.CHANGE_VEHICLE,
            payload,
        });
    };
};

export type CombineAction =
    | TYPES.ChangeScenariosAction
    | AsyncAction<TYPES.IInitState, TYPES.ChangeVehicleAction>
    | AsyncAction<TYPES.IInitState, TYPES.ChangeMapAction>
    | AsyncAction<TYPES.IInitState, ChangeRecorderAction>
    | AsyncAction<TYPES.IInitState, ChangeOperateAction>
    | AsyncAction<TYPES.IInitState, ChangeModeAction>
    | TYPES.ToggleModuleAction
    | TYPES.UpdateStatusAction;
