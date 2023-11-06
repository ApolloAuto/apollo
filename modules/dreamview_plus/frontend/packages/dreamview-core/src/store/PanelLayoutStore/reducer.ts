import { produce } from 'immer';
import { MosaicNode } from 'react-mosaic-component';
import set from 'lodash/set';
import get from 'lodash/get';
import shortUUID from 'short-uuid';
import { PanelType } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import { genereatePanelId, genereateNewMosaicNode } from '@dreamview/dreamview-core/src/util/layout';
import { UPDATE, ADD_PANEL_FROM_OUTSIDE, UPDATE_BY_MODE, REFRESH_PANEL } from './actionTypes';
import { CombineAction, AddPanelFromOutsidePayload, UpdateByNodePayload, RefreshPanelPayload } from './actions';
import { CURRENT_MODE } from '../HmiStore';

export type IInitState = {
    mosaicId: string;
    layout: MosaicNode<string>;
};

const modeLayoutRelation: Record<string, MosaicNode<string>> = {
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
};

export const initState: IInitState = {
    mosaicId: shortUUID.generate(),
    layout: modeLayoutRelation[CURRENT_MODE.DEFAULT],
};

function addPanelFromOutside(state: IInitState, payload: AddPanelFromOutsidePayload) {
    const { path, position, originPanelConfig } = payload;

    const newPanelType = genereatePanelId(originPanelConfig.type);

    const isLeve1 = path.length === 0;

    const hasLayout = !!state.layout;

    const newNode = genereateNewMosaicNode(state.layout, path, position, newPanelType);

    if (!hasLayout) {
        state.layout = newPanelType;
    } else if (isLeve1) {
        state.layout = newNode;
    } else {
        set(state.layout as object, path, newNode) as MosaicNode<string>;
    }
}

function updateByModeHandler(draftState: IInitState, payload: UpdateByNodePayload) {
    const layout = modeLayoutRelation[payload.mode];
    if (layout) {
        draftState.layout = layout;
    }
}

function refreshPanel(draftState: IInitState, payload: RefreshPanelPayload) {
    const { path } = payload;
    const isLeve1 = path.length === 0;
    if (isLeve1) {
        draftState.layout = genereatePanelId(draftState.layout as string);
    } else {
        const newPanelType = genereatePanelId(get(draftState.layout, path));
        set(draftState.layout as object, path, newPanelType) as MosaicNode<string>;
    }
}

export const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case UPDATE:
                draftState.layout = action.payload;
                break;
            case REFRESH_PANEL:
                refreshPanel(draftState, action.payload);
                break;
            case UPDATE_BY_MODE:
                updateByModeHandler(draftState, action.payload);
                break;
            case ADD_PANEL_FROM_OUTSIDE:
                addPanelFromOutside(draftState, action.payload);
                break;
            default:
                break;
        }
    });
