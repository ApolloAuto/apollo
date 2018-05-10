import React from "react";
import ReactDOM from 'react-dom';
import classNames from "classnames";

import helpIcon from "assets/images/icons/help.png";

export default class EditingTip extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            active: false,
            hover: false
        };

        this.toggle = this.toggle.bind(this);
        this.handleMouseIn = this.handleMouseIn.bind(this);
        this.handleMouseOut = this.handleMouseOut.bind(this);

        this.text =
`For Desktop device:
  • To zoom the map: use mouse wheel
  • To move around the map: right-click and drag your mouse,
    or use arrow keys
  • To add a routing point: left-click mouse

For Mobile device:
  • To zoom the map: pinch the screen with two fingers
  • To move around the map: swipe with three fingers
  • To add a routing point: tap with one finger`;
    }

    toggle() {
        const node = ReactDOM.findDOMNode(this);
        this.setState({
            active: !this.state.active
        });
    }

    handleMouseIn() {
        this.setState({
            hover: true
        });
    }

    handleMouseOut() {
        this.setState({
            hover: false
        });
    }

    render() {
        const textStyle = {
            display: this.state.active ? 'block' : 'none'
        };

        return (
            <button className={classNames({
                        "editing-tip" : true,
                        "button" : true,
                        "active": (this.state.active || this.state.hover),
                    })}
                 onClick={this.toggle}
                 onMouseOver={this.handleMouseIn}
                 onMouseOut={this.handleMouseOut}>
                <img src={helpIcon} />
                <p style={textStyle}>{this.text}</p>
            </button>
        );
    }
}