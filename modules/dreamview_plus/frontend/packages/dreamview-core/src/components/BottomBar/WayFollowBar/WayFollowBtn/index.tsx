import React, { useCallback } from 'react';
import { message } from '@dreamview/dreamview-ui';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import Logger from '@dreamview/log';
import { PlayRTKRecordStatus } from '../../util';
import { DynamicEffectButtonMemo, DynamicEffectButtonStatus } from '../../DynamicEffectButton';

const logger = Logger.getInstance('PlayerControlBar');

function WayFollowBtn() {
    const [hmi] = usePickHmiStore();

    const { mainApi, isMainConnected } = useWebSocketServices();

    const handleStartWayFollow = useCallback(() => {
        if (!isMainConnected) {
            return;
        }
        mainApi
            .startPlayRTKRecorder()
            .then(() => {
                logger.debug('start play rtkRecorder');
            })
            .catch((err) => {
                message({ type: 'error', content: err.data.info.message });
            });
    }, [mainApi, isMainConnected]);

    const btnStatus = (() => {
        if ((hmi?.globalComponents.RTKPlayer.processStatus.status as unknown) === PlayRTKRecordStatus.FATAL) {
            return DynamicEffectButtonStatus.START;
        }
        if ((hmi?.globalComponents.RTKPlayer.processStatus.status as unknown) === PlayRTKRecordStatus.OK) {
            return DynamicEffectButtonStatus.RUNNING;
        }
        return DynamicEffectButtonStatus.DISABLE;
    })();

    return (
        <div id='guide-auto-drive-bar'>
            <DynamicEffectButtonMemo
                behavior={{
                    [DynamicEffectButtonStatus.START]: {
                        text: 'START',
                        clickHandler: handleStartWayFollow,
                    },
                    [DynamicEffectButtonStatus.RUNNING]: {
                        text: 'RUNNING',
                    },
                    [DynamicEffectButtonStatus.DISABLE]: {
                        disabledMsg: 'Please record the trajectory first, and select the corresponding record.',
                        text: 'START',
                    },
                }}
                status={btnStatus}
            />
        </div>
    );
}

export const WayFollowBtnMemo = React.memo(WayFollowBtn);
