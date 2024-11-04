import React, { useState } from 'react';
import { IconPark, Popover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import useStyle from './useStyle';
import { useTimeDown, popoverStatus, EMUN_OPERATE_STATUS, EMUN_TIMEDOWN_STATUS } from '../util';

function ResetBtn(props: { disabled: boolean }) {
    const { disabled } = props;
    const { classes, cx } = useStyle();
    const { t } = useTranslation('bottomBar');

    const { isMainConnected, mainApi } = useWebSocketServices();

    const [reset, triggerReset, resetTimerStatus] = useTimeDown(t('resetMsg'), 3);

    const [resetStatus, setResetStatus] = useState<EMUN_OPERATE_STATUS>(EMUN_OPERATE_STATUS.SUCCESS);

    const resetPopoverStatus = popoverStatus(resetStatus, resetTimerStatus);

    // 重置record
    const onReset = () => {
        if (
            resetStatus === EMUN_OPERATE_STATUS.PROGRESSING ||
            resetTimerStatus === EMUN_TIMEDOWN_STATUS.PORGRESSING ||
            disabled
        ) {
            return;
        }
        if (isMainConnected) {
            mainApi
                .resetSimWorld()
                .then(() => {
                    setResetStatus(EMUN_OPERATE_STATUS.SUCCESS);
                    triggerReset(t('resetMsg'), t('resetSuccess'));
                })
                .catch(() => {
                    setResetStatus(EMUN_OPERATE_STATUS.FAILED);
                    triggerReset(t('resetMsg'), t('resetFailed'));
                });
        }
    };

    return (
        <Popover rootClassName={classes[resetPopoverStatus]} placement='topRight' trigger='hover' content={reset}>
            <div onClick={onReset} className={cx({ disabled }, classes['player-reset-btn'])}>
                <IconPark name='IcBottomRailDelete' />
                <span className={classes['player-download-reset-text']}>{t('reset')}</span>
            </div>
        </Popover>
    );
}

export default React.memo(ResetBtn);
