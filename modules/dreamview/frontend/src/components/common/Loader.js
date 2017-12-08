import React from "react";

import RENDERER from "renderer";
import loaderImg from "assets/images/logo_apollo.png";


export default class Loader extends React.Component {
    render() {
        const { height } = this.props;
        const message = OFFLINE_PLAYBACK ? "" : "Please send car initial position and map data.";

        return (
            <div className="loader" style={{height: height}}>
                <div className="img-container">
                    <img src={loaderImg} alt="Loader"/>
                    <div className="status-message">{message}</div>
                </div>
            </div>
        );
    }
}