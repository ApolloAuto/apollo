import React from "react";
import { observer, inject } from "mobx-react";

import RENDERER from "renderer";

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
        // TODO The position of the canvas should not be absolute, maybe.
        return (
            <div id="canvas"
                 className="dreamview-canvas" />
        );
    }
}
