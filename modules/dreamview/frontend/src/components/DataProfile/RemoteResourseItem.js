import React from 'react';
import {inject} from 'mobx-react';
import { Tooltip } from 'antd';
import { RemoteResourseItemStatus, RemoteResourceItemBtn } from './RemoteResourseItemStatus';

/**
 *
 * @param props {{item:
 * {
 * id: string,
 * name: string,
 * status: "notDownloaded" | "toBeUpdated" | "updating" | "downloaded" | "fail",
 * type: '1'|'2'|'3',
 * errorMsg: string
 * }
 * }}
 * @return {JSX.Element}
 * @constructor
 */
function RemoteResourseItem(props) {
  const { id, name, status, errorMsg, type, store } = props.item;
  return (
    <div className='scenario-set-list-item'>
      <Tooltip
        placement='topRight'
        title={name}
      >
        <div className='scenario-set-list-item_name'>
          {/* 产品确认显示名称去除下划线 */}
          {
            name && name.replaceAll('_', ' ')
          }
        </div>
      </Tooltip>
      <RemoteResourseItemStatus status={status} errorMsg={errorMsg}/>
      <RemoteResourceItemBtn
        status={status}
        type={type}
        id={id}
        store={store}/>
    </div>
  );
}

export default inject('store')(RemoteResourseItem);
