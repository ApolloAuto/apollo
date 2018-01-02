import React from "react";
import { inject, observer } from "mobx-react";

@inject("store") @observer
export default class Geolocation extends React.Component {

    render() {
        const { geolocation } = this.props.store;

        const x = geolocation.x ? geolocation.x.toFixed(2) : '?';
        const y = geolocation.y ? geolocation.y.toFixed(2) : '?';

        return (
            <div className="geolocation">
                ( {x}, {y} )
            </div>
        );
    }
}