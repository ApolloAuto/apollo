/* eslint-disable no-restricted-syntax */
/* eslint-disable @typescript-eslint/no-unused-vars */
import { produce } from 'immer';
import { CombineAction, PanelKeyHandler } from './actions';
import {
    ADD_KEY_HANDLER,
    ADD_SELECTED_PANEL_ID,
    DELETE_SELECTED_PANEL_ID,
    ADD_GLOABLE_KEY_HANDLER,
    REMOVE_KEY_HANDLER,
    REMOVE_GLOABLE_KEY_HANDLER,
} from './actionType';
import { KeyHandlers } from '../../components/panels/base/KeyListener';

export type IInitState = {
    selectedPanelIds: Set<string>;
    keyHandlerMap: Map<string, KeyHandlers[]>;
    globalKeyhandlers: Set<KeyHandlers>;
};

export const initState: IInitState = {
    selectedPanelIds: new Set(),
    keyHandlerMap: new Map(),
    globalKeyhandlers: new Set(),
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

function handleAddKeyHandler(draftState: IInitState, panelKeyHandler: PanelKeyHandler) {
    const keyHandlerMap = draftState.keyHandlerMap;
    if (!keyHandlerMap.has(panelKeyHandler.panelId)) {
        keyHandlerMap.set(panelKeyHandler.panelId, [...panelKeyHandler.keyHandlers]);
    } else {
        keyHandlerMap.get(panelKeyHandler.panelId).push(...panelKeyHandler.keyHandlers);
    }
}

function handleRemoveKeyHandler(draftState: IInitState, panelKeyHandler: PanelKeyHandler) {
    const keyHandlerMap = draftState.keyHandlerMap;
    if (keyHandlerMap.has(panelKeyHandler.panelId)) {
        const keyHandlers = keyHandlerMap.get(panelKeyHandler.panelId);
        const targets2Delte = panelKeyHandler.keyHandlers.map(
            (handler) => (handler?.functionalKey ?? '') + handler.keys.join(),
        );
        const hanlders = keyHandlers.filter((handler) => {
            const key = (handler?.functionalKey ?? '') + handler.keys.join();
            return !targets2Delte.includes(key);
        });
        keyHandlerMap.set(panelKeyHandler.panelId, hanlders);
    }
}

function handleAddGloableKeyHandler(draftState: IInitState, gloableKeyHandlers: KeyHandlers[]) {
    // eslint-disable-next-line no-restricted-syntax
    for (const gloableKeyHandler of gloableKeyHandlers) {
        draftState.globalKeyhandlers.add(gloableKeyHandler);
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
            case ADD_KEY_HANDLER:
                handleAddKeyHandler(draftState, action.payload);
                break;
            case ADD_GLOABLE_KEY_HANDLER:
                handleAddGloableKeyHandler(draftState, action.payload);
                break;
            case REMOVE_KEY_HANDLER:
                handleRemoveKeyHandler(draftState, action.payload);
                break;
            default:
                break;
        }
    });
