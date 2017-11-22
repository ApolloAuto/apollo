import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

import MenuItemRadio from 'components/common/MenuItemRadio';

@observer
export default class POI extends React.Component {
    render() {
        const { routeEditingManager, options } = this.props;

        const entries = Object.keys(routeEditingManager.defaultRoutingEndPoint)
            .map(key => {
                return (
                    <MenuItemRadio extraClasses={["poi-button"]}
                                   key={`poi_${key}`} id='poi' title={key}
                                   onClick={() => {
                                        routeEditingManager.addDefaultEndPoint(key);
                                        if (!options.showRouteEditingBar) {
                                            routeEditingManager.sendRoutingRequest();
                                        }
                                        options.showPOI = false;
                                   }}
                                   checked={false}/>
                );
            });

        return (
            <div className="nav-side-menu" id="poi-list">
                <div className="card">
                    <div className="card-header"><span>Point of Interest</span></div>
                    <div className="card-content-row">{entries}</div>
                </div>
            </div>
        );
    }
}