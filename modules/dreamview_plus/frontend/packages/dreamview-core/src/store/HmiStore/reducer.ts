import { produce, enableMapSet } from 'immer';
import { CombineAction } from './actions';
import { ACTIONS, IInitState, CURRENT_MODE } from './actionTypes';
import { reducerHander } from './reducerHandler';

// 使用 Immer 时涉及到了 Map 或 Set 数据结构， 需要开启 MapSet 插件。
enableMapSet();

export const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case ACTIONS.UPDATE_STATUS:
                reducerHander.updateStatus(state, draftState, action.payload);
                break;
            case ACTIONS.TOGGLE_MODULE:
                reducerHander.toggleModule(draftState, action.payload);
                break;
            case ACTIONS.CHANGE_MODE:
                reducerHander.changeMode(action.payload);
                break;
            case ACTIONS.CHANGE_OPERATE:
                reducerHander.changeOperate(action.payload);
                break;
            case ACTIONS.CHANGE_RECORDER:
                reducerHander.changeRecorder(action.payload);
                break;
            case ACTIONS.CHANGE_SCENARIOS:
                reducerHander.changeScenarios(action.payload);
                break;
            case ACTIONS.CHANGE_MAP:
                reducerHander.changeMap(action.payload);
                draftState.envResourcesHDMapDisable = action.payload.mapDisableState;
                break;
            case ACTIONS.CHANGE_VEHICLE:
                reducerHander.changeVehicle(action.payload);
                break;
            default:
                break;
        }
    });

export const initState: IInitState = {
    prevStatus: {},
    modes: [],
    currentMode: CURRENT_MODE.NONE,
    // such as:['Lincoln2017MKZ LGSVL', 'Mkz Example'],
    vehicles: [],
    currentVehicle: '',
    // ['San Mateo', 'Apollo Virutal Map', 'Sunnyvale']
    dockerImage: '',
    maps: [],
    currentMap: '',
    moduleStatus: new Map(),
    modulesLock: new Map(),
    componentStatus: new Map(),

    isVehicleCalibrationMode: false,
    isSensorCalibrationMode: false,
    //  数据包名称1: RECORDER_DOWNLOAD_STATUS.DOWNLOADEND
    records: {},
    currentRecordId: '',
    currentScenarioSetId: '',
    currentScenarioName: '',
    currentScenarioId: '',
    scenarioSet: {},
    otherComponents: {},

    currentRecordStatus: undefined,
    operations: [],
    currentOperation: undefined,

    dataRecorderComponent: {
        processStatus: {
            status: undefined,
        },
        resourceStatus: {
            status: undefined,
        },
    },

    envResourcesHDMapDisable: false,
    backendShutdown: false,
};
