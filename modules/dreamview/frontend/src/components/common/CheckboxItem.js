import React from "react";
import classNames from "classnames";

export default class CheckboxItem extends React.Component {
    render() {
        const { id, title, isChecked, onClick, disabled, extraClasses } = this.props;
        return (
            <ul className={classNames({
                        "disabled": disabled
                    }, extraClasses)}>
                <li id={id} onClick={() => {
                                if (!disabled) {
                                    onClick();
                                }}}>
                    <div className="switch">
                        <input type="checkbox" className="toggle-switch"
                               name={id} checked={isChecked} disabled={disabled} readOnly />
                        <label className="toggle-switch-label" htmlFor={id} />
                    </div>
                    <span>{title}</span>
                </li>
            </ul>
        );
    }
}