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

    removeLastRoutingPoint() {
        RENDERER.removeLastRoutingPoint();
    }

    removeAllRoutingPoints() {
        RENDERER.removeAllRoutingPoints();
    }

    addDefaultEndPoint() {
        RENDERER.addDefaultEndPoint();
    }

    sendRoutingRequest(sendDefaultRoute = false) {
        if (RENDERER.sendRoutingRequest(sendDefaultRoute)){
            this.disableRouteEditing();
        }
    }
}
