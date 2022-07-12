import React from 'react';
import {inject} from 'mobx-react';
import { Tooltip } from 'antd';
import { ScenarioSetItemStatus, ScenarioSetItemBtn } from './ScenarioSetItemStatus';

/**
 *
 * @param props {{item:
 * {
 * scenarioSetId: string,
 * name: string,
 * status: "notDownloaded" | "toBeUpdated" | "updating" | "downloaded" | "fail",
 * errorMsg: string
 * }
 * }}
 * @return {JSX.Element}
 * @constructor
 */
function ScenarioSetItem(props) {
  const { scenarioSetId, name, status, errorMsg, store } = props.item;
  return (
    <div className='scenario-set-list-item'>
      <Tooltip
        placement='topRight'
        title={name}
      >
        <div className='scenario-set-list-item_name'>{name}</div>
      </Tooltip>
      <ScenarioSetItemStatus status={status} errorMsg={errorMsg}/>
      <ScenarioSetItemBtn
        status={status}
        scenarioSetId={scenarioSetId}
        store={store}/>
    </div>
  );
}

export default inject('store')(ScenarioSetItem);
