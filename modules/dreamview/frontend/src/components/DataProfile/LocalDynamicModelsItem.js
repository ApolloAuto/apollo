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
 * currentDynamicModel: string,
 * item: string,
 * store: any,
 * }}
 * @return {JSX.Element}
 * @constructor
 */
function LocalDynamicModelItem(props) {

  const { currentDynamicModel, item, store } = props;

  const { monitor } = store;

  const [deleteConfirmModalVisible, setDeleteConfirmModalVisible] = React.useState(false);

  const showDeleteConfirmModal = () => {
    if (currentDynamicModel === item) {
      return false;
    }
    setDeleteConfirmModalVisible(true);
  };

  const deleteConfirmModalOnCancel = () => {
    setDeleteConfirmModalVisible(false);
  };

  const deleteConfirmModalOnOk = () => {
    WS.deleteDynamicModels(item);
    monitor.insert('INFO', 'Deleted  successfully!', Date.now());
    setDeleteConfirmModalVisible(false);
    // 刷新列表
    WS.getDymaticModelList();
    PLUGIN_WS.getDynamicsModelList();
  };

  return (
    <>
      <div className={classNames(['local-scenario-set-list-item', {
        'local-scenario-set-list-item_selected': currentDynamicModel === item,
      }])}>
        <Radio value={item}>
          {item && item.replaceAll('_', ' ')}
        </Radio>
        {/* 暂时去掉动力学模型的删除功能 */}
        {/* <div */}
        {/*  className='local-scenario-set-list-item_delete' */}
        {/*  onClick={showDeleteConfirmModal} */}
        {/* > */}
        {/*  <img className='local-scenario-set-list-item_delete_icon' src={ProfileDeleteIcon} */}
        {/*       alt='local-profile-scenario-set-delete' /> */}
        {/* </div> */}
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
              Are you sure to delete the dynamic model？
            </h6>
            <div className='local-scenario-set-delete-confirm-modal_content'>
              The current operation only deletes the dynamic model saved locally!
            </div>
          </div>
        </div>
        <div className='local-scenario-set-delete-confirm-modal_button_group'>
          <div className='local-scenario-set-delete-confirm-modal_button_group_item'>
            <div className='local-scenario-set-delete-confirm-modal_button_group_item_button'
                 onClick={deleteConfirmModalOnCancel}
            >cancel
            </div>
          </div>
          <div className='local-scenario-set-delete-confirm-modal_button_group_item'>
            <div className='local-scenario-set-delete-confirm-modal_button_group_item_button'
                 onClick={deleteConfirmModalOnOk}
            >confirm
            </div>
          </div>
        </div>
      </Modal>
    </>
  );
}

export default inject('store')(LocalDynamicModelItem);
