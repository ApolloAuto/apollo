import React, { useEffect, useState } from 'react';
import { Empty, Tag } from 'antd';
import {
  CheckCircleOutlined,
  ExclamationCircleOutlined,
  MinusCircleOutlined,
  SyncOutlined,
} from '@ant-design/icons';
import { vehicleInfoMap } from 'utils/vehicleInfoMap';
import { PLUGIN_WS } from 'store/websocket';
import VehicleListItem from 'components/DataProfile/VehicleListItem';


function V2xList() {
  /**
   * @type {{
   *  v2xId: string,
   *  obu_in: string,
   *  }[]}
   */
  const [v2xInfoList, setV2xInfoList] = useState([]);
  const [v2xUpdateStatus, setV2xUpdateStatus] = useState(0);

  useEffect(() => {
    PLUGIN_WS.getV2xInfo().then((data) => {
      const v2x = [];
      Object.keys(data).forEach((key) => {
        v2x.push({
          v2xId: key,
          ...data[key],
        });
      });
      setV2xInfoList(v2x);
      // 更新完成状态
      setV2xUpdateStatus(1);
    });
  }, []);

  return (<div className='remote-vehicle-list'>
    {/* 更新后 没有存在v2x设备 */}
    {(v2xInfoList.length === 0 && v2xUpdateStatus !== 0) &&
      <div className='remote-vehicle-list_empty'>
        <Empty
          style={{color: '#fff'}}
          image={Empty.PRESENTED_IMAGE_SIMPLE}
          description="There is no v2x under your account."/>
      </div>}
    {/* 更新后 存在v2x设备 */}
    {
      (v2xInfoList.length > 0 && v2xUpdateStatus !== 0) &&
      v2xInfoList.map((item) =>
        <div className='remote-vehicle-list-item-container' key={item.v2xId} >
          <V2xListItem value={item.v2xId} item={item}/>
        </div>
      )
    }
  </div>);
}


function V2xListItem(props) {
  /**
   * @type {{
   *  v2xId: string,
   *  obu_in: string,
   *  }}
   */
  const { item } = props;

  const [status, setStatus] = React.useState('unknown');

  const handleClick = (event) => {
    event.stopPropagation();
    const content = event.target.textContent;
    if (content === 'Reset') {
      setStatus('resetting');
      PLUGIN_WS.resetV2xConf(item.v2xId).then(() => {
        setStatus('clean');
      },
      () => {
        setStatus('error');
      });
    }
    if (content === 'Refresh') {
      setStatus('refreshing');
      PLUGIN_WS.refreshV2xConf(item.v2xId).then(() => {
        setStatus('clean');
      },
      () => {
        setStatus('error');
      });
    }
    if (content === 'Upload') {
      setStatus('uploading');
      PLUGIN_WS.uploadV2xConf(item.v2xId).then(() => {
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
        item.obu_in
      }</div>
      <V2xListItemStatus status={status} />
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
function V2xListItemStatus(props) {
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

export default V2xList;
