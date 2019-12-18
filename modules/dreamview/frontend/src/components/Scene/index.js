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
                this.props.options, this.props.store.cameraData);
    }

    componentWillUpdate(nextProps) {
        if (nextProps.width !== this.props.width ||
            nextProps.height !== this.props.height) {
            // The dimension of the renderer should always be consistent with
            // the dimension of this component.
            RENDERER.updateDimension(nextProps.width, nextProps.height);
        }
    }

    render() {
        const { options, shouldDisplayOnRight } = this.props;

        const shouldDisplayCameraImage = options.showCameraView && !options.showRouteEditingBar;
        const leftPosition = shouldDisplayOnRight ? '50%' : '0%';

        return (
            <React.Fragment>
                {shouldDisplayCameraImage && <img id="camera-image" />}
                <div id="canvas"
                     className="dreamview-canvas"
                     style={{left: leftPosition}}
                     onMouseMove={(event) => {
                        const geo = RENDERER.getGeolocation(event);
                        STORE.setGeolocation(geo);
                     }}>
                    {options.showGeo && <Geolocation />}
                </div>
            </React.Fragment>
        );
    }
}
