import React from 'react';
import { Tag } from 'antd';
import {
  CheckCircleOutlined,
  ExclamationCircleOutlined,
  MinusCircleOutlined,
  SyncOutlined,
} from '@ant-design/icons';
import { vehicleInfoMap } from 'utils/vehicleInfoMap';
import { PLUGIN_WS } from 'store/websocket';

/**
 * 车辆信息
 * @param props {{
 * value: string,
 * item: {
 *       vehicle_id: string,
 *       vin: string,
 *       vtype: string,
 *     }
 * }}
 * @return {JSX.Element}
 * @constructor
 */
function VehicleListItem(props) {
  const { item } = props;

  const [status, setStatus] = React.useState('unknown');

  const handleClick = (event) => {
    event.stopPropagation();
    const content = event.target.textContent;
    if (content === 'Reset') {
      setStatus('resetting');
      PLUGIN_WS.resetVehicleConfig(item.vehicle_id).then(() => {
        setStatus('clean');
      },
      () => {
        setStatus('error');
      });
    }
    if (content === 'Refresh') {
      setStatus('refreshing');
      PLUGIN_WS.refreshVehicleConfig(item.vehicle_id).then(() => {
        setStatus('clean');
      },
      () => {
        setStatus('error');
      });
    }
    if (content === 'Upload') {
      setStatus('uploading');
      PLUGIN_WS.uploadVehicleConfig(item.vehicle_id).then(() => {
        setStatus('clean');
      },
      () => {
        setStatus('error');
      });
    }
  };
  return (
    <div className='vehicle-list-item'>
      <div className='vehicle-list-item_name'>{
        vehicleInfoMap.get(item.vtype)
      }</div>
      <div className='vehicle-list-item_vin'>vin: {item.vin}</div>
      <VehicleListItemStatus status={status} />
      {/* 按钮组 */}
      <div className='vehicle-list-item_btns'>
        <button className='vehicle-list-item_btns_btn' onClick={handleClick}>Refresh</button>
        <button className='vehicle-list-item_btns_btn' onClick={handleClick}>Upload</button>
        <button className='vehicle-list-item_btns_btn' onClick={handleClick}>Reset</button>
      </div>
    </div>
  );
}

/**
 * 更新状态
 * @param props {{
 * status: "refreshing" | "resetting" | "clean" | "unknown" | "error"| "uploading",
 * }}
 * @return {JSX.Element}
 * @constructor
 */
function VehicleListItemStatus(props) {
  const { status } = props;
  let icon = null;
  const color = '#000000';
  switch (status) {
    case 'refreshing':
      icon = <SyncOutlined spin style={{ color: '#1D9063' }} />;
      break;
    case 'resetting':
      icon = <SyncOutlined spin style={{ color: '#1D9063' }} />;
      break;
    case 'clean':
      icon = <CheckCircleOutlined style={{ color: '#1D9063' }} />;
      break;
    case 'unknown':
      icon = <MinusCircleOutlined style={{ color: '#faad14' }} />;
      break;
    case 'error':
      icon = <ExclamationCircleOutlined style={{ color: '#f5222d' }} />;
      break;
    case 'uploading':
      icon = <SyncOutlined spin style={{ color: '#1D9063' }} />;
      break;
    default:
      break;
  }
  return (
    <Tag className='vehicle-list-item_status' icon={icon} color={color}>
      {status.toUpperCase()}
    </Tag>
  );
}

export default VehicleListItem;
