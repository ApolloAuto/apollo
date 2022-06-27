import React from 'react';
import { inject, observer } from 'mobx-react';
import _ from 'lodash';

@inject('store') @observer
export default class RegisterForm extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      editing: false,
    };
    this.vehicleSnEditorRef = React.createRef();
    this.onVehicleSnChange = this.onVehicleSnChange.bind(this);
    this.onVehicleSnEditorKeyUp = this.onVehicleSnEditorKeyUp.bind(this);
    this.onVehicleSnEditorBlur = this.onVehicleSnEditorBlur.bind(this);
    this.onVehicleSnLabelClick = this.onVehicleSnLabelClick.bind(this);
  }

  onVehicleSnChange() {
    const vehicleSn = this.vehicleSnEditorRef.current.value;
    if (vehicleSn !== this.props.vehicleSn) {
      if (_.isFunction(this.props.onVehicleSnChange)) {
        this.props.onVehicleSnChange({
          type: 'change',
          name: 'vehicleSn',
          value: this.vehicleSnEditorRef.current.value,
        });
      }
    }
  }

  onVehicleSnEditorKeyUp(evt) {
    if (evt.key === 'Enter' || evt.keyCode === 13) {
      this.onVehicleSnChange();
      this.setState({ editing: false }, () => {
        this.props.store.setNormalMode();
      });
    }
  }

  onVehicleSnEditorBlur() {
    this.onVehicleSnChange();
    this.setState({ editing: false }, () => {
      this.props.store.setInsertMode();
    });
  }

  onVehicleSnLabelClick() {
    this.setState({ editing: true }, () => {
      this.props.store.setInsertMode();
    });
  }

  renderEditableForm() {
    const { vehicleSn } = this.props;
    return (
      <div className="vin">
        <label className="layout-flex flex-v-center flex-columns20">
          <span className="flex-span1">VIN: </span>
          <input
            className="flex-span18"
            autoFocus
            type="text"
            name="vehicleSn"
            placeholder="Please set your VIN to enable profile feature"
            defaultValue={vehicleSn}
            onChange={_.noop}
            ref={this.vehicleSnEditorRef}
            onKeyUp={this.onVehicleSnEditorKeyUp}
            onBlur={this.onVehicleSnEditorBlur}
          />
        </label>
      </div>
    );
  }

  renderReadonlyForm() {
    const { vehicleSn } = this.props;
    return (
      <div className="vin">
        <label
          className="layout-flex flex-v-center flex-columns20"
          onClick={this.onVehicleSnLabelClick}
        >
          <span className="flex-span1">VIN: </span>
          <span className="flex-span18">
            {vehicleSn ? vehicleSn : 'We Cannot detect your vehicle, please click and set your VIN to enable profile feature'}
          </span>
        </label>
      </div>
    );
  }

  render() {
    if (this.state.editing) {
      return this.renderEditableForm();
    }
    return this.renderReadonlyForm();
  }
}
