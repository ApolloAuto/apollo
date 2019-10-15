import React from "react";
import classNames from "classnames";

export default class RadioItem extends React.Component {
    constructor(props) {
        super(props);

        this.setElementRef = element => {
            this.elementRef = element;
        };
        this.handleKeyPress = this.handleKeyPress.bind(this);
    }

    componentDidMount() {
        if (this.props.autoFocus && this.elementRef) {
            this.elementRef.focus();
        }
    }

    handleKeyPress(event) {
        if (event.key === "Enter") {
            event.preventDefault();
            this.props.onClick();
        }
    }

    render() {
        const { id, title, options, onClick, checked, extraClasses, autoFocus } = this.props;

        return (
            <ul className={classNames("item", extraClasses)}
                tabIndex="0"
                ref={this.setElementRef}
                onKeyPress={this.handleKeyPress}
                onClick={onClick} >
                <li>
                    <input type="radio" name={id} checked={checked} readOnly />
                    <label className="radio-selector-label" htmlFor={title} />
                    <span>{title}</span>
                </li>
            </ul>
        );
    }
}
