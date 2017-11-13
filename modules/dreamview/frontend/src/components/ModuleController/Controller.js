import React from "react";
import { inject, observer } from "mobx-react";

@observer
export default class Controller extends React.Component {
    render() {
        const { id, title, modules, onClick } = this.props;
        return (
            <ul className="controller">
                <li onClick={onClick}>
                    <div className="switch">
                        <input type="checkbox" name={id} className="toggle-switch"
                                checked={modules.get(id)} readOnly/>
                        <label className="toggle-switch-label" htmlFor={id} />
                    </div>
                    <span>{title}</span>
                </li>
            </ul>
        );
    }
}