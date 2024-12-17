import { PayloadAction } from '@dreamview/dreamview-core/src/store/base/Reducer';
import { MosaicNode, MosaicPath } from 'react-mosaic-component';
import { MosaicDropTargetPosition } from 'react-mosaic-component/lib/internalTypes';
import { IPanelMetaInfo } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import {
    UPDATE,
    ADD_PANEL_FROM_OUTSIDE,
    REFRESH_PANEL,
    RESET_LAYOUT,
    EXPAND_MODE_LAYOUT_RELATION,
    INIT_PANEL_LAYOUT,
} from './actionTypes';
import { CURRENT_MODE } from '../HmiStore';

export interface AddPanelFromOutsidePayload {
    mode: CURRENT_MODE;
    path: MosaicPath;
    position: MosaicDropTargetPosition;
    originPanelConfig: IPanelMetaInfo;
    panelId?: string;
}

export interface IResetLayoutByMode {
    mode: CURRENT_MODE;
}

export interface UpdateByNodePayload {
    mode: CURRENT_MODE;
}

export interface RefreshPanelPayload {
    mode: CURRENT_MODE;
    path: MosaicPath;
}

export interface IUpdateLayoutPayload {
    mode: CURRENT_MODE;
    layout: MosaicNode<string>;
}

export interface IExpandModeLayoutRelation {
    mode: string;
    layout: MosaicNode<string>;
}

export type IInitLayoutState = MosaicNode<string>;

export type ICurrentLayoutState = MosaicNode<string>;

export interface IInitLayoutPayload {
    mode: string;
    initLatout: IInitLayoutState;
    currentLayout: ICurrentLayoutState;
}

type UpdateLayoutAction = PayloadAction<typeof UPDATE, IUpdateLayoutPayload>;

type AddPanelFromOutsideAction = PayloadAction<typeof ADD_PANEL_FROM_OUTSIDE, AddPanelFromOutsidePayload>;

type IResetLayoutByModeAction = PayloadAction<typeof RESET_LAYOUT, IResetLayoutByMode>;

type RefreshPanelAction = PayloadAction<typeof REFRESH_PANEL, RefreshPanelPayload>;

type IExpandModeLayoutRelationAction = PayloadAction<typeof EXPAND_MODE_LAYOUT_RELATION, IExpandModeLayoutRelation>;

type IInitPanelLayoutAction = PayloadAction<typeof INIT_PANEL_LAYOUT, IInitLayoutPayload>;

export const update = (payload: IUpdateLayoutPayload): UpdateLayoutAction => ({
    type: UPDATE,
    payload,
});
export const refreshPanel = (payload: RefreshPanelPayload): RefreshPanelAction => ({ type: REFRESH_PANEL, payload });

export const addPanelFromOutside = (payload: AddPanelFromOutsidePayload): AddPanelFromOutsideAction => ({
    type: ADD_PANEL_FROM_OUTSIDE,
    payload,
});

export const resetLayoutByMode = (payload: IResetLayoutByMode): IResetLayoutByModeAction => ({
    type: RESET_LAYOUT,
    payload,
});

export const expandModeLayoutRelation = (payload: IExpandModeLayoutRelation): IExpandModeLayoutRelationAction => ({
    type: EXPAND_MODE_LAYOUT_RELATION,
    payload,
});

export const initPanelLayout = (payload: IInitLayoutPayload): IInitPanelLayoutAction => ({
    type: INIT_PANEL_LAYOUT,
    payload,
});

export type CombineAction =
    | UpdateLayoutAction
    | AddPanelFromOutsideAction
    | RefreshPanelAction
    | IResetLayoutByModeAction
    | IExpandModeLayoutRelationAction
    | IInitPanelLayoutAction;
