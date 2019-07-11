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

    renderTipContent() {
        return (
            <table>
                <thead>
                    <tr>
                        <th />
                        <th>Zooming</th>
                        <th>Panning</th>
                    </tr>
                </thead>
                <tbody>
                    <tr>
                        <td>Mouse</td>
                        <td>Scroll your mouse wheel</td>
                        <td>Right-click and drag</td>
                    </tr>
                    <tr>
                        <td>Trackpad</td>
                        <td>Pinch or spread with two fingers</td>
                        <td>Press and swipe with two fingers</td>
                    </tr>
                    <tr>
                        <td>Keyboard</td>
                        <td>-</td>
                        <td>Press arrow keys</td>
                    </tr>
                </tbody>
            </table>
        );
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
                <div style={textStyle} className="tip-popup">
                    <div className="title">To navigate through the map</div>
                    {this.renderTipContent()}
                </div>
            </button>
        );
    }
}