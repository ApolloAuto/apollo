import { produce, enableMapSet } from 'immer';
import { CombineAction } from './actions';
import { ACTIONS, IInitState, CURRENT_MODE, ENUM_DATARECORD_PROCESS_STATUS } from './actionTypes';
import { reducerHander } from './reducerHandler';

// 使用 Immer 时涉及到了 Map 或 Set 数据结构， 需要开启 MapSet 插件。
enableMapSet();

export const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case ACTIONS.UPDATE_STATUS:
                reducerHander.updateStatusSimp(state, draftState, action.payload);
                break;
            case ACTIONS.TOGGLE_MODULE:
                reducerHander.toggleModule(draftState, action.payload);
                break;
            case ACTIONS.CHANGE_MODE:
                reducerHander.updateCurrentMode(draftState, action.payload);
                break;
            case ACTIONS.CHANGE_OPERATE:
                reducerHander.changeOperate(draftState, action.payload);
                break;
            case ACTIONS.CHANGE_RECORDER:
                reducerHander.changeRecorder(action.payload);
                break;
            case ACTIONS.CHANGE_RTK_RECORDER:
                reducerHander.changeRTKRecorder(action.payload);
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
    dynamicModels: [],
    currentMode: CURRENT_MODE.NONE,
    // such as:['Lincoln2017MKZ LGSVL', 'Mkz Example'],
    vehicles: [],
    currentVehicle: '',
    // ['San Mateo', 'Apollo Virutal Map', 'Sunnyvale']
    dockerImage: '',
    maps: [],
    currentMap: '',
    modules: new Map(),
    modulesLock: new Map(),

    //  数据包名称1: RECORDER_DOWNLOAD_STATUS.DOWNLOADEND
    records: {},
    rtkRecords: [],
    currentRecordId: '',
    currentRtkRecordId: '',
    currentScenarioSetId: '',
    currentScenarioName: '',
    currentScenarioId: '',
    currentDynamicModel: '',
    scenarioSet: {},
    otherComponents: {},
    simOtherComponents: {},
    currentRecordStatus: undefined,
    operations: [],
    currentOperation: undefined,

    globalComponents: {
        DataRecorder: {
            processStatus: {
                message: '',
                status: undefined,
            },
            resourceStatus: {
                message: '',
                status: undefined,
            },
        },
        RTKPlayer: {
            processStatus: {
                message: '',
                status: undefined,
            },
            summary: {
                message: '',
                status: undefined,
            },
        },
        RTKRecorder: {
            processStatus: {
                message: '',
                status: undefined,
            },
            summary: {
                message: '',
                status: undefined,
            },
        },
    },

    Terminal: {
        processStatus: {
            message: '',
            status: ENUM_DATARECORD_PROCESS_STATUS.FATAL,
        },
        summary: {
            message: '',
            status: ENUM_DATARECORD_PROCESS_STATUS.FATAL,
        },
    },

    envResourcesHDMapDisable: false,
    backendShutdown: false,
};
