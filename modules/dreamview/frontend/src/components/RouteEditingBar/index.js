import React from "react";
import { inject, observer } from "mobx-react";

import Image from "components/common/Image";
import logoApollo from "assets/images/logo_apollo.png";
import EditingPanel from "components/RouteEditingBar/EditingPanel";


@observer
class EditorExitButton extends React.Component {
    render() {
        const { label, onClick } = this.props;
        return (
            <button onClick={onClick}
                    className="exit-button">X</button>
        );
    }
}

@inject("store") @observer
export default class RouteEditingMenu extends React.Component {
    render() {
        const { routeEditingManager } = this.props.store;

        return (
            <div className="route-editing-bar">
                <Image image={logoApollo} className="apollo-logo" />
                <EditingPanel clickRemoveLast={() => {
                                    routeEditingManager.removeLastRoutingPoint();
                              }}
                              clickRemoveAll={() => {
                                    routeEditingManager.removeAllRoutingPoints();
                              }}
                              clickSendRoute={() => {
                                    routeEditingManager.sendRoutingRequest();
                              }}
                />
                <EditorExitButton onClick={ () => {
                    routeEditingManager.disableRouteEditing();
                }}/>
            </div>
        );
    }
}
