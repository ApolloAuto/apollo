import React from "react";
import classNames from "classnames";

export default class CheckboxItem extends React.Component {
    render() {
        const { id, title, isChecked, onClick, disabled, extraClasses } = this.props;
        return (
            <ul className={classNames({
                            "item": true,
                            "disabled": disabled
                        }, extraClasses)}>
                <li id={id}
                    tabIndex="0"
                    onClick={() => {
                        if (!disabled) {
                            onClick();
                        }
                    }}
                    onKeyPress={(event) => {
                        event.preventDefault();
                        if (event.key === 'Enter' || event.key === ' ') {
                            onClick();
                        }
                    }}>
                    <div className="switch">
                        <input type="checkbox" className="toggle-switch"
                            name={id} checked={isChecked} disabled={disabled} readOnly />
                        <label className="toggle-switch-label" htmlFor={id} />
                    </div>
                    {title && <span>{title}</span>}
                </li>
            </ul>
        );
    }
}