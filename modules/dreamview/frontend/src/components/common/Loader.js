import React from "react";

import RENDERER from "renderer";
import loaderImg from "assets/images/loader_apollo.gif";


export default class Loader extends React.Component {

    render() {
        const message = "Please send car initial position and map data.";

        return (
            <div className="loader">
                <div className="img-container">
                    <img src={loaderImg} alt="Loader"/>
                    <div className="status-message">{message}</div>
                </div>
            </div>
        );
    }
}