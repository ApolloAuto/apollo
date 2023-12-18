import React, { useState } from 'react';
import { IconIcBottomRailDownload } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import Popover from '@dreamview/dreamview-core/src/components/CustomPopover';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import useStyle from './useStyle';
import { useTimeDown, popoverStatus, EMUN_OPERATE_STATUS, EMUN_TIMEDOWN_STATUS } from '../util';

function DumpBtn(props: { disabled: boolean }) {
    const { disabled } = props;
    const { classes, cx } = useStyle();
    const { t } = useTranslation('bottomBar');

    const { isMainConnected, mainApi } = useWebSocketServices();

    const [dumpStatus, setDumpStatus] = useState<EMUN_OPERATE_STATUS>(EMUN_OPERATE_STATUS.SUCCESS);

    const [dump, triggerDump, dumpTimerStatus] = useTimeDown(t('dumpMsg', { path: '/tmp' }), 3);

    const dumpPopoverStatus = popoverStatus(dumpStatus, dumpTimerStatus);

    // 下载record
    const onDownload = () => {
        if (dumpStatus === EMUN_OPERATE_STATUS.PROGRESSING || dumpTimerStatus === EMUN_TIMEDOWN_STATUS.PORGRESSING) {
            return;
        }
        setDumpStatus(EMUN_OPERATE_STATUS.PROGRESSING);
        if (isMainConnected) {
            mainApi
                .dumpFrame()
                .then(() => {
                    setDumpStatus(EMUN_OPERATE_STATUS.SUCCESS);
                    triggerDump(t('dumpMsg', { path: '/tmp' }), t('dumpSuccess'));
                })
                .catch(() => {
                    triggerDump(t('dumpMsg', { path: '/tmp' }), t('dumpFailed'));
                    setDumpStatus(EMUN_OPERATE_STATUS.FAILED);
                });
        }
    };

    return (
        <Popover rootClassName={classes[dumpPopoverStatus]} placement='topRight' trigger='hover' content={dump}>
            <div onClick={onDownload} className={cx({ disabled }, classes['player-download-btn'])}>
                <IconIcBottomRailDownload />
                <span className={classes['player-download-btn-text']}>{t('dump')}</span>
            </div>
        </Popover>
    );
}

export default React.memo(DumpBtn);
