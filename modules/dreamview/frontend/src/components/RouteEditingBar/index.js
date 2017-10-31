import React from "react";
import { inject, observer } from "mobx-react";

import Image from "components/common/Image";
import logoApollo from "assets/images/logo_apollo.png";
import EditingPanel from "components/RouteEditingBar/EditingPanel";
import EditingTip from "components/RouteEditingBar/EditingTip";
import helpIcon from "assets/images/icons/help.png";
import POI from "components/SideBar/POI";

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
        const { routeEditingManager, options } = this.props.store;

        return (
            <div>
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
                                  options.toggleShowPOI();
                              }}
                />
                <EditorExitButton onClick={ () => {
                    routeEditingManager.disableRouteEditing();
                }}/>
            </div>
            <div className="sidebar">
                {options.showPOI ? <POI routeEditingManager={routeEditingManager}
                    options={options} /> : <div/>}
            </div>
            </div>
        );
    }
}
