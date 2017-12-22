import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";


class SideBarSubButton extends React.Component {
    render() {
        const { disabled, onClick, active, label, extraClasses, iconSrc } = this.props;
        return (
            <button onClick={onClick}
                    disabled={disabled}
                    className={classNames({
                            "sub-button": true,
                            "sub-button-active": active,
                        }, extraClasses)}>
                <div className="label">{label}</div>
            </button>
        );
    }
}

export default class ButtonPanel extends React.Component {
    render() {
        const { enablePOI, onPOI, showPOI } = this.props;

        return (
            <div>
                <SideBarSubButton label="Default Routing"
                               disabled={!enablePOI}
                               onClick={onPOI}
                               active={showPOI} />
            </div>
        );
    }
}
