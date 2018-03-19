import React from "react";
import classNames from "classnames";


export default class SubButton extends React.Component {
    render() {
        const { enablePanel, onPanel, showPanel, panelLabel, extraClasses } = this.props;
        return (
            <button onClick={onPanel}
                    disabled={!enablePanel}
                    className={classNames({
                            "sub-button": true,
                            "sub-button-active": showPanel,
                        }, extraClasses)}>
                <div className="label">{panelLabel}</div>
            </button>
        );
    }
}