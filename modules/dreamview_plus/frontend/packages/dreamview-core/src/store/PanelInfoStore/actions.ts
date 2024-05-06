import { KeyHandlers } from '../../components/panels/base/KeyListener';
import { PayloadAction } from '../base/Reducer';
import {
    ADD_KEY_HANDLER,
    ADD_SELECTED_PANEL_ID,
    DELETE_SELECTED_PANEL_ID,
    ADD_GLOABLE_KEY_HANDLER,
    REMOVE_KEY_HANDLER,
    REMOVE_GLOABLE_KEY_HANDLER,
} from './actionType';

export type PanelKeyHandler = {
    panelId: string;
    keyHandlers: KeyHandlers[];
};

type AddSelectedPanelIdAction = PayloadAction<typeof ADD_SELECTED_PANEL_ID, string>;

type DeleteSelectedPanelIdAction = PayloadAction<typeof DELETE_SELECTED_PANEL_ID, string>;

type AddKeyHandlerAction = PayloadAction<typeof ADD_KEY_HANDLER, PanelKeyHandler>;

type RemoveKeyHandlerAction = PayloadAction<typeof REMOVE_KEY_HANDLER, PanelKeyHandler>;

type AddGloableKeyHandlerAction = PayloadAction<typeof ADD_GLOABLE_KEY_HANDLER, KeyHandlers[]>;

type RemoveGloableKeyHandlerAction = PayloadAction<typeof REMOVE_GLOABLE_KEY_HANDLER, KeyHandlers[]>;

export const addSelectedPanelId = (payload: string): AddSelectedPanelIdAction => ({
    type: ADD_SELECTED_PANEL_ID,
    payload,
});

export const deleteSelectedPanelId = (payload: string): DeleteSelectedPanelIdAction => ({
    type: DELETE_SELECTED_PANEL_ID,
    payload,
});

export const addKeyHandler = (payload: PanelKeyHandler): AddKeyHandlerAction => ({
    type: ADD_KEY_HANDLER,
    payload,
});

export const removeKeyHandler = (payload: PanelKeyHandler): RemoveKeyHandlerAction => ({
    type: REMOVE_KEY_HANDLER,
    payload,
});

export const addGloableKeyHandler = (payload: KeyHandlers[]): AddGloableKeyHandlerAction => ({
    type: ADD_GLOABLE_KEY_HANDLER,
    payload,
});

export const removeGloableKeyHandler = (payload: KeyHandlers[]): RemoveGloableKeyHandlerAction => ({
    type: REMOVE_GLOABLE_KEY_HANDLER,
    payload,
});

export type CombineAction =
    | AddSelectedPanelIdAction
    | DeleteSelectedPanelIdAction
    | AddKeyHandlerAction
    | AddGloableKeyHandlerAction
    | RemoveKeyHandlerAction
    | RemoveGloableKeyHandlerAction;
