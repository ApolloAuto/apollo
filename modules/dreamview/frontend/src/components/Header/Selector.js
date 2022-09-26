import React from 'react';

export default class Selector extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
      name: props.name,
      value: props.currentOption,
    };

    this.onChangeHandler = this.onChangeHandler.bind(this);
  }

  onChangeHandler(event) {
    this.setState({ value: event.target.value });
    this.props.onChange(event);
  }

  componentWillReceiveProps(nextProps) {
    if (nextProps.currentOption !== this.props.currentOption) {
      this.setState({ value: nextProps.currentOption });
    }
  }

  render() {
    const {
      name, options, currentOption, onChange,
    } = this.props;

    this.entries = options.map((key) => (
            <option value={key} key={key}>{key}</option>
    ));
    this.entries.unshift(
            <option key="none" value="none" disabled>
                {`-- ${this.state.name} --`}
            </option>,
    );

    return (
            <div className="header-item selector">
                <span className="arrow" />
                <select onChange={this.onChangeHandler} value={this.state.value}>
                    {this.entries}
                </select>
            </div>
    );
  }
}
