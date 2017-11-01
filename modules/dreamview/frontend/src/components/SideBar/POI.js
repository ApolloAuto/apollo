import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

@observer
class MenuItemRadio extends React.Component {
    render() {
        const {id, title, routeEditingManager, options} = this.props;
        return (
            <ul>
                <li id={title} onClick={() => {
                    routeEditingManager.addDefaultEndPoint(title);
                    if (!routeEditingManager.inEditingView) {
                        routeEditingManager.sendRoutingRequest();
                    }
                    options.showPOI = false;
                }}>
                    <input type="radio" name={id} id={title}
                           checked={routeEditingManager.poi_name === title} readOnly/>
                    <label id="radio-selector-label" htmlFor={title} />
                    <span>{title}</span>
                </li>
            </ul>
        );
    }
}

@observer
export default class POI extends React.Component {
    render() {
        const { routeEditingManager, options } = this.props;
        const entries = Object.keys(routeEditingManager.defaultRoutingEndPoint)
            .map(key => {
                return (
                    <MenuItemRadio key={`poi_${key}`} id='poi' title={key}
                    routeEditingManager={routeEditingManager} options={options}/>
                );
            });

        return (
            <div className="nav-side-menu" id="poi-list">
                {entries}
            </div>
        );
    }
}
