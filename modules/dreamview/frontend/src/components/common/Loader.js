import React from "react";

import classNames from "classnames";

import RENDERER from "renderer";
import loaderImg from "assets/images/logo_apollo.png";
import loaderGif from "assets/images/loader_apollo.gif";

export default class Loader extends React.PureComponent {
    render() {
        const { extraClasses, offlineViewErr } = this.props;

        let message = "Please send car initial position and map data.";
        if (OFFLINE_PLAYBACK) {
            message = offlineViewErr || "Loading ....";
        }
        const imgSrc = OFFLINE_PLAYBACK ? loaderGif : loaderImg;

        return (
            <div className="loader">
                <div className={classNames("img-container", extraClasses)}>
                    <img src={imgSrc} alt="Loader" />
                    <div className={offlineViewErr ? 'error-message' : 'status-message'}>
                        {message}
                    </div>
                </div>
            </div>
        );
    }
}
