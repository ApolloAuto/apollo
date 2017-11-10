import React from "react";

import syncupIcon from "assets/images/icons/syncup.png";

export default class VideoControls extends React.Component {

    render() {
        const { showSyncup, onSyncup, onClose } = this.props;

        const syncupButton = showSyncup
                ? ( <button className="syncup" onClick={onSyncup}>
                        <img src={syncupIcon} />
                    </button> )
                : ( null );

        return  (
            <div className="controls">
                {syncupButton}
                <button className="close" onClick={onClose}>x</button>
            </div>
        );
    }
}
