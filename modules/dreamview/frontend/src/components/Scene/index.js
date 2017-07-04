import React from "react";
import { observer, inject } from "mobx-react";

import RENDERER from "renderer";

export default class Scene extends React.Component {
    componentDidMount() {
        RENDERER.initialize("canvas", this.props.width, this.props.height);
        window.addEventListener("blur", () => {
            RENDERER.stopAnimate();
        });
        window.addEventListener("focus", () => {
            RENDERER.startAnimate();
        });
    }

    componentWillUpdate() {
        // The dimension of the renderer should always be consistent with
        // the dimension of this component.
        RENDERER.updateDimension(this.props.width, this.props.height);
    }

    render() {
        // TODO The position of the canvas should not be absolute, maybe.
        return (
            <div id="canvas"
                 className="dreamview-canvas" />
        );
    }
}
