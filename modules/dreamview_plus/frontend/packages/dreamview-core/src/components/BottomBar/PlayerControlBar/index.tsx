import React, { useCallback, useEffect, useMemo } from 'react';
import Logger from '@dreamview/log';
import { IconIcPlay, IconIcStopPlaying } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import Popover from '@dreamview/dreamview-core/src/components/CustomPopover';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { useHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import useStyle from './useStyle';
import { PlayRecordStatus, useKeyDownEvent } from '../util';
import Progress from './Progress';
import DumpBtn from '../Operate/Dump';
import ResetBtn from '../Operate/Reset';
import Record from '../Operate/Record';

const logger = Logger.getInstance('PlayerControlBar');

function formatTs(v: number) {
    if (!v) return 0;

    return Math.floor(Math.round(v * 100)) / 100;
}

// 播放进度条组件
function PlayerControlBar() {
    const { isMainConnected, mainApi } = useWebSocketServices();
    const { t } = useTranslation('bottomBar');
    const [hmi] = useHmiStore();
    const recordInfo = useMemo(
        () => ({
            totalTimeS: formatTs(hmi?.records[hmi?.currentRecordStatus?.currentRecordId]),
            currTimeS: formatTs(hmi?.currentRecordStatus?.currTimeS || 0),
            playRecordStatus: hmi?.currentRecordStatus?.playRecordStatus || PlayRecordStatus.CLOSED,
            disabled: !hmi.currentRecordStatus?.currentRecordId,
        }),
        [hmi?.currentRecordStatus, isMainConnected],
    );

    const { classes, cx } = useStyle();

    // 播放状态:是否在播放中
    const isInPlaying = recordInfo.playRecordStatus === PlayRecordStatus.RUNNING;

    // 播放进度
    const onChange = useCallback(
        (progress: number) => {
            if (isMainConnected) {
                mainApi.resetRecordProgress({
                    progress,
                });
            }
        },
        [isMainConnected, recordInfo.totalTimeS],
    );

    useEffect(() => {
        if (!recordInfo.currTimeS || !recordInfo.totalTimeS) return;
        if (!isMainConnected) return;
        // 服务端需要知道视频播放完成了，这里调用接口通知服务端
        if (recordInfo.currTimeS === recordInfo.totalTimeS) {
            mainApi.stopRecord();
        }
    }, [recordInfo.currTimeS, isMainConnected, recordInfo.totalTimeS]);

    // 停止播放record
    const onEnd = useCallback(() => {
        if (recordInfo.disabled) {
            return;
        }
        if (!mainApi?.ctrRecorderAction) {
            return;
        }
        mainApi.ctrRecorderAction('pause', recordInfo.currTimeS);
    }, [recordInfo.disabled, mainApi, recordInfo.currTimeS]);

    // 开始播放record
    const onStart = useCallback(() => {
        if (recordInfo.disabled) {
            return;
        }
        if (!mainApi) {
            return;
        }
        if (recordInfo.playRecordStatus === PlayRecordStatus.CLOSED) {
            mainApi.startPlayRecorder().then(() => {
                logger.debug('start play recorder');
            });
        }
        if (recordInfo.playRecordStatus === PlayRecordStatus.PAUSED) {
            mainApi.ctrRecorderAction('continue', recordInfo.currTimeS);
        }
    }, [mainApi, recordInfo.disabled, recordInfo.playRecordStatus, recordInfo.currTimeS]);

    const playIc = isInPlaying ? (
        <IconIcPlay disabled={recordInfo.disabled} onClick={onEnd} className='ic-play-btn' />
    ) : (
        <IconIcStopPlaying onClick={onStart} className='ic-play-btn' />
    );

    const popoverPlayIc = recordInfo.disabled ? (
        <Popover placement='topLeft' trigger='hover' content={t('recordMsg')}>
            {playIc}
        </Popover>
    ) : (
        playIc
    );

    useKeyDownEvent(onStart, onEnd);

    return (
        <div className={cx(classes['player-controlbar-container'], { [classes.disabled]: recordInfo.disabled })}>
            {popoverPlayIc}
            <span className='player-progress-text'>
                {recordInfo.currTimeS}
                &nbsp;/&nbsp;
                {recordInfo.totalTimeS}
            </span>
            <Progress
                onChange={onChange}
                progress={recordInfo.currTimeS}
                duration={recordInfo.totalTimeS}
                className='player-progress'
            />
            <Record />
            <DumpBtn disabled={recordInfo.disabled} />
            <ResetBtn disabled={recordInfo.disabled} />
        </div>
    );
}

export default React.memo(PlayerControlBar);
