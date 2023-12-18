import { produce } from 'immer';
import { MosaicNode } from 'react-mosaic-component';
import set from 'lodash/set';
import get from 'lodash/get';
import shortUUID from 'short-uuid';
import { PanelType } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import { genereatePanelId, genereateNewMosaicNode } from '@dreamview/dreamview-core/src/util/layout';
import { UPDATE, ADD_PANEL_FROM_OUTSIDE, REFRESH_PANEL, RESET_LAYOUT } from './actionTypes';
import { CombineAction, AddPanelFromOutsidePayload, IResetLayoutByMode, RefreshPanelPayload } from './actions';
import { CURRENT_MODE } from '../HmiStore';

export type IInitState = {
    [key in CURRENT_MODE]: {
        mosaicId: string;
        layout: MosaicNode<string>;
    };
};

type IModeLayoutRelation = {
    [key in CURRENT_MODE]: MosaicNode<string>;
};

const modeLayoutRelation: () => IModeLayoutRelation = () => ({
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
});

export const mosaicId = shortUUID.generate();

export const initState: IInitState = Object.entries(modeLayoutRelation()).reduce(
    (result, [key, layout]) => ({
        ...result,
        [key]: {
            mosaicId,
            layout,
        },
    }),
    {} as IInitState,
);

function addPanelFromOutside(state: IInitState, payload: AddPanelFromOutsidePayload) {
    const { path, position, originPanelConfig, panelId, mode } = payload;

    const newPanelType = panelId || genereatePanelId(originPanelConfig.type);

    const isLeve1 = !path || path.length === 0;

    const hasLayout = !!state[mode].layout;

    const newNode = genereateNewMosaicNode(state[mode].layout, path, position, newPanelType);

    if (!hasLayout) {
        state[mode].layout = newPanelType;
    } else if (isLeve1) {
        state[mode].layout = newNode;
    } else {
        set(state[mode].layout as object, path, newNode) as MosaicNode<string>;
    }
}

function refreshPanel(draftState: IInitState, payload: RefreshPanelPayload) {
    const { path, mode } = payload;
    const isLeve1 = path.length === 0;
    if (isLeve1) {
        draftState[mode].layout = genereatePanelId(draftState[mode].layout as string);
    } else {
        const newPanelType = genereatePanelId(get(draftState[mode].layout, path));
        set(draftState[mode].layout as object, path, newPanelType) as MosaicNode<string>;
    }
}

function resetLayoutByModeHandler(draftState: IInitState, payload: IResetLayoutByMode) {
    draftState[payload.mode].layout = modeLayoutRelation()[payload.mode];
}

export const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        const { mode } = action.payload;
        switch (action.type) {
            case UPDATE:
                draftState[mode].layout = action.payload.layout;
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
            default:
                break;
        }
    });
