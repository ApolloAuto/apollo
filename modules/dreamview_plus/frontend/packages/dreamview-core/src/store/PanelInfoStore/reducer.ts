import { produce } from 'immer';
import { CombineAction } from './actions';
import { ADD_SELECTED_PANEL_ID, DELETE_SELECTED_PANEL_ID } from './actionType';

export type IInitState = {
    selectedPanelIds: Set<string>;
};

export const initState: IInitState = {
    selectedPanelIds: new Set(),
};

function handleAddSelectedPanelId(draftState: IInitState, panelId: string) {
    const selectedPanelIds = draftState.selectedPanelIds;
    if (selectedPanelIds.size !== 0) {
        selectedPanelIds.clear();
    }
    selectedPanelIds.add(panelId);
}

function handleDeleteSelectedPanelId(draftState: IInitState, panelId: string) {
    const selectedPanelIds = draftState.selectedPanelIds;
    if (selectedPanelIds.has(panelId)) {
        selectedPanelIds.delete(panelId);
    }
}

export const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case ADD_SELECTED_PANEL_ID:
                handleAddSelectedPanelId(draftState, action.payload);
                break;
            case DELETE_SELECTED_PANEL_ID:
                handleDeleteSelectedPanelId(draftState, action.payload);
                break;
            default:
                break;
        }
    });
