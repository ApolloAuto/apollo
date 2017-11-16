import React from "react";


export default class MenuItemRadio extends React.Component {
    render() {
        const {id, title, options, onClick, checked, extraClasses} = this.props;
        return (
            <ul className={extraClasses}>
                <li onClick={onClick}>
                    <input type="radio" name={id} checked={checked} readOnly/>
                    <label className="radio-selector-label" htmlFor={title} />
                    <span>{title}</span>
                </li>
            </ul>
        );
    }
}