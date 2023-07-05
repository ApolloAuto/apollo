import React from 'react';
import { Radio } from 'antd';
import classNames from 'classnames';
import { Modal } from 'antd';

import { inject } from 'mobx-react';

import ProfileDeleteIcon from 'assets/images/icons/profile_delete.png';
import ProfileWarningIcon from 'assets/images/icons/profile_warning.png';
import WS, { PLUGIN_WS } from 'store/websocket';

/**
 * 展示本地场景集
 * @param props {{
 *   currentScenarioSet: string,
 *   item: {
 *     name: string,
 * scenarioSetId: string,
 * scenarios: {
 * mapName: string,
 * scenarioId: string,
 * scenarioName: string,
 *   }[]
 *   }
 * }}
 * @return {JSX.Element}
 * @constructor
 */
function LocalScenarioSetItem(props) {

  const { currentScenarioSetId, item, store } = props;

  const { monitor } = store;

  const [deleteConfirmModalVisible, setDeleteConfirmModalVisible] = React.useState(false);

  const showDeleteConfirmModal = () => {
    setDeleteConfirmModalVisible(true);
  };

  const deleteConfirmModalOnCancel = () => {
    setDeleteConfirmModalVisible(false);
  };

  const deleteConfirmModalOnOk = () => {
    WS.deleteScenarioSet(item.scenarioSetId);
    monitor.insert('INFO', 'Deleted  successfully!', Date.now());
    setDeleteConfirmModalVisible(false);
    // 刷新列表
    PLUGIN_WS.getScenarioSetList();
  };

  return (
    <>
      <div className={classNames(['local-scenario-set-list-item', {
        'local-scenario-set-list-item_selected': currentScenarioSetId === item.scenarioSetId,
      }])}>
        <Radio value={item.scenarioSetId}>{item.name}</Radio>
        <div
          className='local-scenario-set-list-item_delete'
          onClick={showDeleteConfirmModal}
        >
          <img className='local-scenario-set-list-item_delete_icon' src={ProfileDeleteIcon}
               alt='local-profile-scenario-set-delete' />
        </div>
      </div>
      <Modal
        wrapClassName='local-scenario-set-delete-confirm-modal'
        centered
        closable={false}
        footer={null}
        visible={deleteConfirmModalVisible}
        onOk={deleteConfirmModalOnOk}
        onCancel={deleteConfirmModalOnCancel}
      >
        <div className='local-scenario-set-delete-confirm-modal_body'>
          <div className='local-scenario-set-delete-confirm-modal_icons'>
            <img src={ProfileWarningIcon} alt='local-scenario-set-delete-confirm' />
          </div>
          <div>
            <h6 className='local-scenario-set-delete-confirm-modal_title'>
              Are you sure to delete the scenario set ？
            </h6>
            <div className='local-scenario-set-delete-confirm-modal_content'>
              The current operation only deletes the scenario set saved locally,
              and the scenario set stored in Apollo studio cloud will still be retained.
            </div>
          </div>
        </div>
        <div className='local-scenario-set-delete-confirm-modal_button_group'>
          <div className='local-scenario-set-delete-confirm-modal_button_group_item'>
            <div className='local-scenario-set-delete-confirm-modal_button_group_item_button'
                 onClick={deleteConfirmModalOnCancel}
            >取消
            </div>
          </div>
          <div className='local-scenario-set-delete-confirm-modal_button_group_item'>
            <div className='local-scenario-set-delete-confirm-modal_button_group_item_button'
                 onClick={deleteConfirmModalOnOk}
            >确定
            </div>
          </div>
        </div>
      </Modal>
    </>
  );
}

export default inject('store')(LocalScenarioSetItem);
