import React from "react";

export default class RadioItem extends React.Component {
    constructor(props) {
        super(props);

        this.setElementRef = (element) => {
            this.elementRef = element;
        };
    }

    componentDidMount() {
        if (this.props.autoFocus && this.elementRef) {
            this.elementRef.focus();
        }
    }

    render() {
        const { id, title, options, onClick, checked, extraClasses, autoFocus } = this.props;

        return (
            <ul className={extraClasses}
                tabIndex="0"
                ref={this.setElementRef}
                onKeyPress={onClick}
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
