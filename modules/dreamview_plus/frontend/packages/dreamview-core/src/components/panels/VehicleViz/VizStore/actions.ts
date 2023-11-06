import { PayloadAction } from '@dreamview/dreamview-core/src/store/base/Reducer';
import { RoutingEditor } from '@dreamview/dreamview-carviz';
import { RouteManager } from '@dreamview/dreamview-core/src/components/panels/VehicleViz/RoutingEditing/RouteManager';
import { INIT_ROUTING_EDITOR, INIT_ROUTE_MANAGER } from './actionTypes';

export type IInitState = {
    routingEditor?: RoutingEditor;
    routeManager?: RouteManager;
};

type InitRoutingEditorAction = PayloadAction<typeof INIT_ROUTING_EDITOR, IInitState>;

type InitRouteMangerAction = PayloadAction<typeof INIT_ROUTE_MANAGER, IInitState>;

export const initRoutingEditor = (payload: IInitState): InitRoutingEditorAction => ({
    type: INIT_ROUTING_EDITOR,
    payload,
});

export const initRouteManager = (payload: IInitState): InitRouteMangerAction => ({
    type: INIT_ROUTE_MANAGER,
    payload,
});

export type CombineAction = InitRoutingEditorAction | InitRouteMangerAction;
