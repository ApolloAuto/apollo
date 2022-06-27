import React from 'react';
import { CONFIGURATION_WS } from 'store/websocket';
import _ from 'lodash';

export default class VehicleProfile extends React.Component {
  requestVehicleProfileTarballDownload(vehicleId, profileId) {
    return CONFIGURATION_WS.requestVehicleProfileTarballDownload(vehicleId, profileId);
  }

  cancelVehicleProfileTarballDownload(vehicleId, profileId) {
    return CONFIGURATION_WS.cancelVehicleProfileTarballDownload(vehicleId, profileId);
  }

  requestVehicleProfileTarballReset(vehicleId, profileId) {
    return CONFIGURATION_WS.requestVehicleProfileTarballReset(vehicleId, profileId);
  }

  cancelVehicleProfileTarballReset(vehicleId, profileId) {
    return CONFIGURATION_WS.cancelVehicleProfileTarballReset(vehicleId, profileId);
  }

  requestVehicleProfileTarballUpload(vehicleId, profileId) {
    return CONFIGURATION_WS.requestVehicleProfileTarballUpload(vehicleId, profileId);
  }

  cancelVehicleProfileTarballUpload(vehicleId, profileId) {
    return CONFIGURATION_WS.cancelVehicleProfileTarballUpload(vehicleId, profileId);
  }

  renderAction(idx, label, op, disabled = false) {
    return (
      <button
        key={idx}
        className="command-button"
        onClick={op}
        disabled={disabled}
      >
        {label}
      </button>
    );
  }

  render() {
    const { profileId, vehicleId, vehicleType, status } = this.props;
    const STATUS_COLORS = {
      SYNCING: 'rgba(235, 194, 164)',
      CLEAN: 'rgba(158, 212, 206)',
      UNKNOWN: 'rgba(224, 230, 236)',
    };
    // TODO: Support multiple operations
    const STATUS_ACTIONS = {
      CLEAN: [
        {
          label: 'Refresh',
          op: _.partial(this.requestVehicleProfileTarballDownload, vehicleId, profileId),
          disabled: false,
        },
        {
          label: 'Upload',
          op: _.partial(this.requestVehicleProfileTarballUpload, vehicleId, profileId),
          disabled: false,
        },
        {
          label: 'Reset',
          op: _.partial(this.requestVehicleProfileTarballReset, vehicleId, profileId),
          disabled: false,
        },
      ],
      SYNCING: [
        {
          label: 'Cancel',
          op: _.partial(this.cancelVehicleProfileTarballDownload, vehicleId, profileId),
          disabled: false,
        },
      ],
      UNKNOWN: [
        {
          label: 'Refresh',
          op: _.partial(this.requestVehicleProfileTarballDownload, vehicleId, profileId),
          disabled: false,
        },
        {
          label: 'Reset',
          op: _.partial(this.requestVehicleProfileTarballReset, vehicleId, profileId),
          disabled: false,
        },
      ],
      DOWNLOADING: [
        {
          label: 'Cancel',
          op: _.partial(this.cancelVehicleProfileTarballDownload, vehicleId, profileId),
          disabled: false,
        },
      ],
      UPLOADING: [
        {
          label: 'Cancel',
          op: _.partial(this.cancelVehicleProfileTarballUpload, vehicleId, profileId),
          disabled: false,
        },
      ],
      RESETING: [
        {
          label: 'Cancel',
          op: _.partial(this.cancelVehicleProfileTarballReset, vehicleId, profileId),
          disabled: false,
        },
      ],
    };
    const VTYPE_NAMES = {
      1: 'D-KIT Lite',
      2: 'D-KIT Standard',
      3: 'D-KIT Advanced Ne-s',
      4: 'D-KIT Advanced Sne-r',
      5: 'D-KIT Lite S',
      6: 'D-KIT Standard S',
      7: 'D-KIT Challenge',
      65535: 'Other',
    };
    return (
      <label
        className="layout-flex flex-v-center flex-columns12 vehicle"
      >
        <input className="flex-span1" type="radio" name="id" value={profileId} />
        <div
          className="flex-span4"
        >
          {VTYPE_NAMES[vehicleType]}
        </div>
        <div
          className="flex-span3 layout-flex flex-v-center flex-columns7 status"
        >
          <div className="layout-flex flex-v-center flex-span2 icon">
            <svg version="1.1" width="25px" height="20px">
              <circle cx="10" cy="10" r="7" fill={STATUS_COLORS[status]} />
            </svg>
          </div>
          <div className="flex-span5 name">
            {status}
          </div>
        </div>
        <div
          className="flex-span3 layout-flex"
        >
          {(STATUS_ACTIONS[status] || []).map((item, idx) => (
            this.renderAction(idx, item.label, item.op, item.disabled)
          ))}
        </div>
      </label>
    );
  }
}
