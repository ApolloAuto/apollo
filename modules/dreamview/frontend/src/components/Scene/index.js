import React from "react";
import { observer, inject } from "mobx-react";
import classNames from "classnames";

import Geolocation from "components/Scene/Geolocation";
import RENDERER from "renderer";
import STORE from "store";

@inject("store") @observer
export default class Scene extends React.Component {
    componentDidMount() {
        RENDERER.initialize("canvas", this.props.width, this.props.height,
                this.props.options);
    }

    componentWillUpdate(nextProps) {
        // The dimension of the renderer should always be consistent with
        // the dimension of this component.
        RENDERER.updateDimension(nextProps.width, nextProps.height);
    }

    render() {
        const {invisible, options} = this.props;

        return (
            <div id = "canvas"
                 className={classNames({
                            "dreamview-canvas" : true,
                             "hidden" : invisible})}
                 onMouseMove={(event) => {
                    const geo = RENDERER.getGeolocation(event);
                    STORE.setGeolocation(geo);
                 }}>
                {options.showGeo && <Geolocation />}
            </div>
        );
    }
}
