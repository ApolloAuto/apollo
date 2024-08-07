import Logger from '@dreamview/log';
import { message } from '@dreamview/dreamview-ui';
import { TFunction } from 'i18next';
import {
    ACTIONS,
    ChangeModeAction,
    ChangeOperateAction,
    ChangeDynamicAction,
    ChangeRecorderAction,
    ChangeRTKRecorderAction,
    CURRENT_MODE,
} from './actionTypes';
import * as TYPES from './actionTypes';
import { MainApi, OtherApi } from '../../services/api';
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
    callback?: (mode: CURRENT_MODE) => void,
): AsyncAction<TYPES.IInitState, ChangeModeAction> => {
    noop();
    return async (_dispatch, state) => {
        logger.debug('changeMode', { state, payload });
        await mainApi.changeSetupMode(payload);
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
        await mainApi.resetSimWorld();
        dispatch({
            type: ACTIONS.CHANGE_RECORDER,
            payload,
        });
    };
};

export const changeRTKRecord = (
    mainApi: MainApi,
    payload: TYPES.ChangeRTKRecorderPayload,
): AsyncAction<TYPES.IInitState, ChangeRTKRecorderAction> => {
    noop();
    return async (dispatch, state) => {
        logger.debug('changeRTKRecorder', { state, payload });
        await mainApi.changeRTKRecord(payload);
        dispatch({
            type: ACTIONS.CHANGE_RTK_RECORDER,
            payload,
        });
    };
};

export const changeDynamic = (
    mainApi: MainApi,
    payload: TYPES.ChangeDynamicPayload,
): AsyncAction<TYPES.IInitState, ChangeDynamicAction> => {
    noop();
    return async (_dispatch, state) => {
        logger.debug('changeDynamic', { state, payload });
        await mainApi.changeDynamicModel(payload);
    };
};

export const changeScenarios = (
    otherApi: OtherApi,
    mainApi: MainApi,
    payload: TYPES.ChangeScenariosPayload,
): AsyncAction<TYPES.IInitState, TYPES.ChangeScenariosAction> => {
    noop();
    return async (dispatch, state) => {
        logger.debug('changeScenarios', { state, payload });
        const res = await otherApi.changeScenarios(payload.scenarioId, payload.scenariosSetId);
        if (res) {
            await mainApi.changeMap(res.currentScenarioMap).then((status) => {
                if (!status.isOk) {
                    message({
                        type: 'error',
                        content: 'Auto-switching map failed',
                        key: 'MODE_SETTING_SCENARIO_CHANGE_ERROR',
                    });
                }
                otherApi.resetScenario();
            });
        }
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

export const updateCurrentMode: any = (mode: TYPES.ChangeModePayload) => ({
    type: ACTIONS.CHANGE_MODE,
    payload: mode,
});

export const updateCurrentOperate: any = (mode: TYPES.ChangeOperatePayload) => ({
    type: ACTIONS.CHANGE_OPERATE,
    payload: mode,
});

export type CombineAction =
    | AsyncAction<TYPES.IInitState, TYPES.ChangeScenariosAction>
    | AsyncAction<TYPES.IInitState, TYPES.ChangeVehicleAction>
    | AsyncAction<TYPES.IInitState, TYPES.ChangeMapAction>
    | AsyncAction<TYPES.IInitState, ChangeRecorderAction>
    | AsyncAction<TYPES.IInitState, ChangeRTKRecorderAction>
    | AsyncAction<TYPES.IInitState, ChangeOperateAction>
    | AsyncAction<TYPES.IInitState, ChangeModeAction>
    | TYPES.ToggleModuleAction
    | TYPES.UpdateStatusAction;
