import { produce } from 'immer';
import { CombineAction, IInitState } from './actions';
import { INIT_ROUTING_EDITOR, INIT_ROUTE_MANAGER } from './actionTypes';

export const initState: IInitState = {
    routingEditor: null,
    routeManager: null,
};

export const reducer = (state: IInitState, action: CombineAction) =>
    produce(state, (draftState: IInitState) => {
        switch (action.type) {
            case INIT_ROUTING_EDITOR:
                draftState.routingEditor = action.payload.routingEditor;
                break;
            case INIT_ROUTE_MANAGER:
                draftState.routeManager = action.payload.routeManager;
                break;
            default:
                break;
        }
    });
