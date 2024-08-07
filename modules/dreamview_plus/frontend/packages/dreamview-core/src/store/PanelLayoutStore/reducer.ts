import { produce } from 'immer';
import { MosaicNode } from 'react-mosaic-component';
import set from 'lodash/set';
import get from 'lodash/get';
import shortUUID from 'short-uuid';
import { PanelType } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import { LocalStorage } from '@dreamview/dreamview-core/src/util/storageManager';
import { genereatePanelId, genereateNewMosaicNode } from '@dreamview/dreamview-core/src/util/layout';
import {
    EXPAND_MODE_LAYOUT_RELATION,
    UPDATE,
    ADD_PANEL_FROM_OUTSIDE,
    REFRESH_PANEL,
    RESET_LAYOUT,
} from './actionTypes';
import { CombineAction, AddPanelFromOutsidePayload, IResetLayoutByMode, RefreshPanelPayload } from './actions';
import { CURRENT_MODE } from '../HmiStore';

type ILayout = {
    [key in CURRENT_MODE]: {
        mosaicId: string;
        layout: MosaicNode<string>;
    };
};

type IExpandLayout = {
    [key: string]: {
        mosaicId: string;
        layout: MosaicNode<string>;
    };
};

type IModeLayoutRelation = {
    [key in CURRENT_MODE]: MosaicNode<string>;
};

type IExpandModeLayoutRelation = {
    [key: string]: MosaicNode<string>;
};

export type IInitState = {
    layout: ILayout & IExpandLayout;
    modeLayoutRelation: IModeLayoutRelation & IExpandModeLayoutRelation;
};

const initModeLayoutRelation: () => IModeLayoutRelation = () => ({
    [CURRENT_MODE.NONE]: null,
    [CURRENT_MODE.PERCEPTION]: {
        first: {
            first: {
                first: genereatePanelId(PanelType.CameraView),
                second: genereatePanelId(PanelType.PointCloud),
                direction: 'row',
            },
            second: genereatePanelId(PanelType.VehicleViz),
            direction: 'column',
        },
        second: {
            first: genereatePanelId(PanelType.Console),
            second: genereatePanelId(PanelType.ModuleDelay),
            direction: 'column',
        },
        direction: 'row',
        splitPercentage: 70,
    },
    [CURRENT_MODE.DEFAULT]: {
        direction: 'row',
        first: genereatePanelId(PanelType.VehicleViz),
        second: {
            first: genereatePanelId(PanelType.DashBoard),
            second: {
                first: genereatePanelId(PanelType.ModuleDelay),
                second: genereatePanelId(PanelType.Console),
                direction: 'column',
                splitPercentage: 40,
            },
            direction: 'column',
            splitPercentage: 33,
        },
        splitPercentage: 66,
    },
    [CURRENT_MODE.PNC]: {
        first: {
            first: genereatePanelId('vehicleViz'),
            second: {
                first: genereatePanelId('console'),
                second: genereatePanelId('moduleDelay'),
                direction: 'row',
            },
            direction: 'column',
            splitPercentage: 66,
        },
        second: {
            first: genereatePanelId('dashBoard'),
            second: genereatePanelId('pncMonitor'),
            direction: 'column',
            splitPercentage: 42,
        },
        direction: 'row',
        splitPercentage: 66,
    },
    [CURRENT_MODE.VEHICLE_TEST]: {
        first: {
            direction: 'column',
            first: genereatePanelId(PanelType.VehicleViz),
            splitPercentage: 66,
            second: {
                second: genereatePanelId(PanelType.Components),
                first: genereatePanelId(PanelType.ModuleDelay),
                direction: 'row',
            },
        },
        second: {
            direction: 'column',
            first: genereatePanelId(PanelType.DashBoard),
            second: genereatePanelId(PanelType.Console),
            splitPercentage: 43,
        },
        splitPercentage: 72,
        direction: 'row',
    },
    [CURRENT_MODE.MAP_COLLECT]: {
        direction: 'row',
        first: genereatePanelId(PanelType.MapCollect),
        second: {
            direction: 'column',
            // first: {
            //     direction: 'column',
            //     first: genereatePanelId(PanelType.PointCloud),
            //     second: genereatePanelId(PanelType.VehicleViz),
            // },
            first: genereatePanelId(PanelType.PointCloud),
            second: genereatePanelId(PanelType.Components),
        },
        splitPercentage: 75,
    },
    [CURRENT_MODE.MAP_EDITOR]: genereatePanelId('MapEditor'),
    [CURRENT_MODE.CAMERA_CALIBRATION]: genereatePanelId('CameraCalibration'),
    [CURRENT_MODE.LiDAR_CALIBRATION]: genereatePanelId('LidarCalibration'),
    [CURRENT_MODE.DYNAMICS_CALIBRATION]: genereatePanelId('DynamicsCalibration'),
    [CURRENT_MODE.CANBUS_DEBUG]: genereatePanelId('CanbusDebug'),
});

export const mosaicId = (() => {
    const uid = shortUUID.generate();
    const storageManager = new LocalStorage('mosaicId');
    if (storageManager.get()) {
        return storageManager.get();
    }
    storageManager.set(uid);
    return uid;
})();

const relations = initModeLayoutRelation();
export const initState: IInitState = {
    layout: Object.entries(relations).reduce(
        (result, [key, layout]) => ({
            ...result,
            [key]: {
                mosaicId,
                layout,
            },
        }),
        {} as IInitState['layout'],
    ),
    modeLayoutRelation: relations,
};

function addPanelFromOutside(state: IInitState, payload: AddPanelFromOutsidePayload) {
    const { path, position, originPanelConfig, panelId, mode } = payload;

    const newPanelType = panelId || genereatePanelId(originPanelConfig.type);

    const isLeve1 = !path || path.length === 0;

    if (!state.layout[mode]) {
        state.layout[mode] = {
            mosaicId,
            layout: '',
        };
    }

    const hasLayout = !!state.layout[mode].layout;

    const newNode = genereateNewMosaicNode(state.layout[mode].layout, path, position, newPanelType);

    if (!hasLayout) {
        state.layout[mode].layout = newPanelType;
    } else if (isLeve1) {
        state.layout[mode].layout = newNode;
    } else {
        set(state.layout[mode].layout as object, path, newNode) as MosaicNode<string>;
    }
}

function refreshPanel(draftState: IInitState, payload: RefreshPanelPayload) {
    const { path, mode } = payload;
    const isLeve1 = path.length === 0;
    if (isLeve1) {
        draftState.layout[mode].layout = genereatePanelId(draftState.layout[mode].layout as string);
    } else {
        const newPanelType = genereatePanelId(get(draftState.layout[mode].layout, path));
        set(draftState.layout[mode].layout as object, path, newPanelType) as MosaicNode<string>;
    }
}

function resetLayoutByModeHandler(draftState: IInitState, payload: IResetLayoutByMode) {
    draftState.layout[payload.mode].layout = initModeLayoutRelation()[payload.mode];
}

function expandModeLayoutRelation(draftState: IInitState, payload: { mode: string; layout: MosaicNode<string> }) {
    draftState.modeLayoutRelation[payload.mode as string] = payload.layout;

    if (!draftState.layout) {
        draftState.layout[payload.mode as string] = {
            mosaicId,
            layout: payload.layout,
        };
    }
}

export const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case UPDATE:
                if (!draftState.layout[action.payload.mode]) {
                    draftState.layout[action.payload.mode] = {
                        mosaicId,
                        layout: action.payload.layout,
                    };
                } else {
                    draftState.layout[action.payload.mode].layout = action.payload.layout;
                }
                break;
            case REFRESH_PANEL:
                refreshPanel(draftState, action.payload);
                break;
            case ADD_PANEL_FROM_OUTSIDE:
                addPanelFromOutside(draftState, action.payload);
                break;
            case RESET_LAYOUT:
                resetLayoutByModeHandler(draftState, action.payload);
                break;
            case EXPAND_MODE_LAYOUT_RELATION:
                expandModeLayoutRelation(draftState, action.payload);
                break;
            default:
                break;
        }
    });
