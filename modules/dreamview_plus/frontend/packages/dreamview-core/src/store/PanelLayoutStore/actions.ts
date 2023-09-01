import { PayloadAction } from '@dreamview/dreamview-core/src/store/base/Reducer';
import { MosaicNode, MosaicPath } from 'react-mosaic-component';
import { MosaicDropTargetPosition } from 'react-mosaic-component/lib/internalTypes';
import { IPanelMetaInfo } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import { UPDATE, ADD_PANEL_FROM_OUTSIDE, UPDATE_BY_MODE, REFRESH_PANEL } from './actionTypes';

export interface AddPanelFromOutsidePayload {
    path: MosaicPath;
    position: MosaicDropTargetPosition;
    originPanelConfig: IPanelMetaInfo;
}

export interface UpdateByNodePayload {
    mode: 'default' | 'prection' | string;
}

export interface RefreshPanelPayload {
    path: MosaicPath;
}

type UpdateLayoutAction = PayloadAction<typeof UPDATE, MosaicNode<string>>;

type UpdateByModeAction = PayloadAction<typeof UPDATE_BY_MODE, UpdateByNodePayload>;

type AddPanelFromOutsideAction = PayloadAction<typeof ADD_PANEL_FROM_OUTSIDE, AddPanelFromOutsidePayload>;

type RefreshPanelAction = PayloadAction<typeof REFRESH_PANEL, RefreshPanelPayload>;

export const update = (payload: MosaicNode<string>): UpdateLayoutAction => ({ type: UPDATE, payload });

export const updateByMode = (payload: UpdateByNodePayload): UpdateByModeAction => ({ type: UPDATE_BY_MODE, payload });

export const refreshPanel = (payload: RefreshPanelPayload): RefreshPanelAction => ({ type: REFRESH_PANEL, payload });

export const addPanelFromOutside = (payload: AddPanelFromOutsidePayload): AddPanelFromOutsideAction => ({
    type: ADD_PANEL_FROM_OUTSIDE,
    payload,
});

export type CombineAction = UpdateLayoutAction | AddPanelFromOutsideAction | UpdateByModeAction | RefreshPanelAction;
