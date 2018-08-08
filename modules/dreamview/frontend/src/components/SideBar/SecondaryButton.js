import React from "react";
import classNames from "classnames";


export default class SecondaryButton extends React.PureComponent {
    render() {
        const { disabled, onClick, active, panelLabel, extraClasses } = this.props;
        return (
            <button onClick={onClick}
                    disabled={disabled}
                    className={classNames({
                            "sub-button": true,
                            "sub-button-active": active,
                        }, extraClasses)}>
                <div className="label">{panelLabel}</div>
            </button>
        );
    }
}