import Logger from '@dreamview/log';
import { message } from '@dreamview/dreamview-ui';
import { TFunction } from 'i18next';
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
        await mainApi.resetSimWorld();
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

export const changeScenarios = (
    mainApi: MainApi,
    payload: TYPES.ChangeScenariosPayload,
): AsyncAction<TYPES.IInitState, TYPES.ChangeScenariosAction> => {
    noop();
    return async (dispatch, state) => {
        logger.debug('changeScenarios', { state, payload });
        await mainApi.changeScenarios(payload.scenarioId, payload.scenariosSetId);
        dispatch({
            type: ACTIONS.CHANGE_SCENARIOS,
            payload,
        });
    };
};

export const changeMap = (
    mainApi: MainApi,
    mapId: string,
    translation: TFunction,
): AsyncAction<TYPES.IInitState, TYPES.ChangeMapAction> => {
    noop();
    return async (dispatch, state) => {
        logger.debug('changeMap', { state, mapId });
        try {
            message({ type: 'loading', content: translation('mapLoading'), key: 'MODE_SETTING_MAP_CHANGE_LOADING' });
            dispatch({
                type: ACTIONS.CHANGE_MAP,
                payload: { mapSetId: mapId, mapDisableState: true },
            });
            await mainApi.changeMap(mapId);
            message.destory('MODE_SETTING_MAP_CHANGE_LOADING');
            dispatch({
                type: ACTIONS.CHANGE_MAP,
                payload: { mapSetId: mapId, mapDisableState: false },
            });
        } catch (error) {
            message.destory('MODE_SETTING_MAP_CHANGE_LOADING');
            dispatch({
                type: ACTIONS.CHANGE_MAP,
                payload: { mapSetId: mapId, mapDisableState: false },
            });
        }
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
    | AsyncAction<TYPES.IInitState, TYPES.ChangeScenariosAction>
    | AsyncAction<TYPES.IInitState, TYPES.ChangeVehicleAction>
    | AsyncAction<TYPES.IInitState, TYPES.ChangeMapAction>
    | AsyncAction<TYPES.IInitState, ChangeRecorderAction>
    | AsyncAction<TYPES.IInitState, ChangeOperateAction>
    | AsyncAction<TYPES.IInitState, ChangeModeAction>
    | TYPES.ToggleModuleAction
    | TYPES.UpdateStatusAction;
