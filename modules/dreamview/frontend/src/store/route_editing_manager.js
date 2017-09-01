import { observable, action } from "mobx";

import RENDERER from "renderer";

export default class RouteEditingManager {

    @observable inEditingView = false;

    @action enableRouteEditing() {
        this.inEditingView = true;
        RENDERER.enableRouteEditing();
    }

    @action disableRouteEditing() {
        this.inEditingView = false;
        RENDERER.disableRouteEditing();
    }

    @action removeLastRoutingPoint() {
        RENDERER.removeLastRoutingPoint();
    }

    @action removeAllRoutingPoints() {
        RENDERER.removeAllRoutingPoints();
    }

    @action sendRoutingRequest(sendDefaultRoute = false) {
        RENDERER.sendRoutingRequest(sendDefaultRoute);
    }
}
