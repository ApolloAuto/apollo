import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

import RadioItem from 'components/common/RadioItem';

@observer
export default class POI extends React.Component {
    render() {
        const { routeEditingManager, options, inNavigationMode } = this.props;

        const entries = Object.keys(routeEditingManager.defaultRoutingEndPoint)
            .map((key, index) => {
                return (
                    <RadioItem extraClasses={["poi-button"]}
                               key={`poi_${key}`} id='poi' title={key}
                               onClick={() => {
                                    routeEditingManager.addDefaultEndPoint(key, inNavigationMode);
                                    if (!options.showRouteEditingBar) {
                                        routeEditingManager.sendRoutingRequest(inNavigationMode);
                                    }
                                    options.showPOI = false;
                               }}
                               autoFocus={index === 0}
                               checked={false}/>
                );
            });

        return (
            <div className="tool-view-menu" id="poi-list">
                <div className="card">
                    <div className="card-header"><span>Point of Interest</span></div>
                    <div className="card-content-row">{entries}</div>
                </div>
            </div>
        );
    }
}