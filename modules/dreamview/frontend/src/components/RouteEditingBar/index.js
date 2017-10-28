import React from "react";
import { inject } from "mobx-react";

import Image from "components/common/Image";
import logoApollo from "assets/images/logo_apollo.png";
import EditingPanel from "components/RouteEditingBar/EditingPanel";
import EditingTip from "components/RouteEditingBar/EditingTip";
import helpIcon from "assets/images/icons/help.png";


class EditorExitButton extends React.Component {
    render() {
        const { label, onClick } = this.props;
        return (
            <button onClick={onClick}
                    className="exit-button">X</button>
        );
    }
}

@inject("store")
export default class RouteEditingMenu extends React.Component {
    render() {
        const { routeEditingManager } = this.props.store;

        return (
            <div className="route-editing-bar">
                <EditingTip />
                <EditingPanel clickRemoveLast={() => {
                                    routeEditingManager.removeLastRoutingPoint();
                              }}
                              clickRemoveAll={() => {
                                    routeEditingManager.removeAllRoutingPoints();
                              }}
                              clickSendRoute={() => {
                                    routeEditingManager.sendRoutingRequest();
                              }}
                              clickAddDefaultEndPoint={() => {
                                    routeEditingManager.addDefaultEndPoint();
                              }}
                />
                <EditorExitButton onClick={ () => {
                    routeEditingManager.disableRouteEditing();
                }}/>
            </div>
        );
    }
}
