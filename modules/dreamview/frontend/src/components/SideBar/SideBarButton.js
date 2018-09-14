import React from "react";
import classNames from "classnames";

export default class SideBarButton extends React.PureComponent {
    render() {
        const { type, label, iconSrc, hotkey,
                active, disabled, extraClasses, onClick } = this.props;

        const isSubButton = type === "sub";
        const tooltip = hotkey ? `${label} (${hotkey})` : label;

        return (
            <button
                onClick={onClick}
                disabled={disabled}
                data-for={"sidebar-button"}
                data-tip={tooltip}
                className={classNames({
                        'button': !isSubButton,
                        'button-active': !isSubButton && active,
                        'sub-button': isSubButton,
                        'sub-button-active': isSubButton && active,
                    },
                    extraClasses
                )} >
                {iconSrc && <img src={iconSrc} className="icon" />}
                <div className="label">{label}</div>
            </button>
        );
    }
}
