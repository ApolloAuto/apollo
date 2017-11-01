import React from "react";

import WS from "store/websocket";

export default class Selector extends React.Component {
    constructor(props) {
        super(props);

        this.state = {
            value: props.currentOption,
        };

        this.onChangeHandler = this.onChangeHandler.bind(this);

        this.entries = this.props.options.map(key => {
            return (
                <option value={key} key={key}>{key}</option>
            );
        });
        this.entries.unshift(
            <option key="none" value="none" disabled style={{"display":"none"}}>
                -- select an option --
            </option>
        );
    }

    onChangeHandler(event) {
        this.setState({value: event.target.value});
        this.props.onChange(event);
    }

    render() {
        const { options, currentOption, onChange } = this.props;

        return (
            <div className="selector">
                <span className="arrow"></span>
                <select onChange={this.onChangeHandler} value={this.state.value}>
                    {this.entries}
                </select>
            </div>
        );
    }
}