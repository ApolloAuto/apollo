import { PayloadAction } from '../base/Reducer';
import { ADD_SELECTED_PANEL_ID, DELETE_SELECTED_PANEL_ID } from './actionType';

type AddSelectedPanelIdAction = PayloadAction<typeof ADD_SELECTED_PANEL_ID, string>;

type DeleteSelectedPanelIdAction = PayloadAction<typeof DELETE_SELECTED_PANEL_ID, string>;

export const addSelectedPanelId = (payload: string): AddSelectedPanelIdAction => ({
    type: ADD_SELECTED_PANEL_ID,
    payload,
});

export const deleteSelectedPanelId = (payload: string): DeleteSelectedPanelIdAction => ({
    type: DELETE_SELECTED_PANEL_ID,
    payload,
});

export type CombineAction = AddSelectedPanelIdAction | DeleteSelectedPanelIdAction;
