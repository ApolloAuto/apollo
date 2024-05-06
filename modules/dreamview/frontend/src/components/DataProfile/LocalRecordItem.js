import React from 'react';
import { Radio } from 'antd';
import classNames from 'classnames';
import { Modal } from 'antd';

import { inject } from 'mobx-react';

import ProfileDeleteIcon from 'assets/images/icons/profile_delete.png';
import ProfileWarningIcon from 'assets/images/icons/profile_warning.png';
import ProfileDownloadIcon from 'assets/images/icons/download.png';
import WS, { PLUGIN_WS } from 'store/websocket';

import PlayIcon from 'assets/images/icons/play.png';
import PauseIcon from 'assets/images/icons/pause.png';

/**
 * 展示本地场景集
 * @param props {{
 * currentRecordId: string,
 * item: string,
 * recordStatus: 0 | 1,
 * updateRecordList: () => void,
 * changeRecord: (item:string)=> void,
 * store: any,
 * }}
 * @return {JSX.Element}
 * @constructor
 */
function LocalDynamicModelItem(props) {

  const { currentRecordId, item, store, changeRecord, recordStatus, updateRecordList } = props;

  // 下载中 刷新列表
  if (recordStatus === 0) {
    updateRecordList();
  }

  const { monitor } = store;

  const [deleteConfirmModalVisible, setDeleteConfirmModalVisible] = React.useState(false);

  const showDeleteConfirmModal = () => {
    if (currentRecordId === item) {
      return false;
    }
    setDeleteConfirmModalVisible(true);
  };

  const deleteConfirmModalOnCancel = () => {
    setDeleteConfirmModalVisible(false);
  };

  const deleteConfirmModalOnOk = () => {
    WS.deleteRecord(item);
    monitor.insert('INFO', 'Deleted  successfully!', Date.now());
    setDeleteConfirmModalVisible(false);
    // 刷新列表
    // WS.loadLocalRecords();
    PLUGIN_WS.getRecordList();
  };

  return (
    <>
      <div className={classNames(['local-record-list-item', {
        'local-record-list-item_selected': currentRecordId === item,
      }])}
      onClick={() => {
        // 下载完成 可以播放
        if (recordStatus === 1) {
          changeRecord(item);
        }
      }}
      >
        <div>{item}</div>
        {/*删除按钮*/}
        { (recordStatus === 1) &&
          (currentRecordId !== item) &&
          (<div
          className='local-record-list-item_delete'
          onClick={(e) => {
            e.stopPropagation();
            showDeleteConfirmModal();
          }}
        >
          <img className='local-record-list-item_delete_icon' src={ProfileDeleteIcon}
               alt='local-profile-record-delete' />
        </div>)}
        {/*播放控制按钮*/}
        {(recordStatus === 1) &&
          (currentRecordId !== item) &&
          (<div
          className='local-record-list-item_ctrl'
          onClick={(e) => {
            e.stopPropagation();
            changeRecord(item);
          }}
        >
          <img className='local-record-list-item_ctrl_icon' src={PlayIcon}
               alt='local-profile-record-play' />
        </div>)}

        {(recordStatus === 1) &&
          (currentRecordId === item) && (<div
          className='local-record-list-item_ctrl'
          onClick={(e) => {
            e.stopPropagation();
            WS.stopRecord();
          }}
        >
          <img className='local-record-list-item_ctrl_icon' src={PauseIcon}
               alt='local-profile-record-play' />
        </div>)}

        {/* 下载中 */}
        {(recordStatus === 0) && (<div
          className='local-record-list-item_download'
        >
          <img className='local-record-list-item_download_icon'
               src={ProfileDownloadIcon}
               alt='local-profile-record-download' />
        </div>)}

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
              Are you sure to delete the record？
            </h6>
            <div className='local-scenario-set-delete-confirm-modal_content'>
              The current operation only deletes the record saved locally!
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
