import React from "react";

import Image from "components/common/Image";
import logoApollo from "assets/images/logo_apollo.png";
import HMIControls from "components/Header/HMIControls";

export default class Header extends React.Component {
    render() {
        return (
            <header className="header">
                <Image image={logoApollo} className="apollo-logo" />
                {!OFFLINE_PLAYBACK && <HMIControls />}
            </header>
        );
    }
}
