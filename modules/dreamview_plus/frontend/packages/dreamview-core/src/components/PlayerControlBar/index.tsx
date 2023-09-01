import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import Logger from '@dreamview/log';
import { apollo } from '@dreamview/dreamview';
import {
    IconIcPlay,
    IconIcStopPlaying,
    IconIcBottomRailDownload,
    IconIcBottomRailDelete,
} from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import Popover from '@dreamview/dreamview-core/src/components/CustomPopover';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { useHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import useStyle from './useStyle';
import Progress from './Progress';

import IRecordInfo = apollo.dreamview.IRecordInfo;

const logger = Logger.getInstance('PlayerControlBar');

export enum PlayRecordStatus {
    RUNNING = 'RUNNING',
    PAUSED = 'PAUSED',
    CLOSED = 'CLOSED',
}
function formatTs(v: number) {
    if (!v) return 0;

    return Math.floor(Math.round(v * 100)) / 100;
}
enum EMUN_OPERATE_STATUS {
    SUCCESS,
    PROGRESSING,
    FAILED,
}
enum EMUN_TIMEDOWN_STATUS {
    INIT,
    PORGRESSING,
}
function useTimeDown(
    defaultText: string,
    time: number,
): [string, (i: string, p: string) => void, EMUN_TIMEDOWN_STATUS] {
    const [text, setText] = useState<string>(defaultText);
    const running = useRef<EMUN_TIMEDOWN_STATUS>(EMUN_TIMEDOWN_STATUS.INIT);
    const timer = useRef<number>();

    useEffect(() => {
        setText(defaultText);
    }, [defaultText]);

    const changeIntoProgressing = () => {
        running.current = EMUN_TIMEDOWN_STATUS.PORGRESSING;
    };

    const changeIntoInit = () => {
        running.current = EMUN_TIMEDOWN_STATUS.INIT;
    };

    const clearTimer = () => {
        clearInterval(timer.current);
    };

    const trigger = useCallback((init: string, progressText: string) => {
        if (running.current === EMUN_TIMEDOWN_STATUS.PORGRESSING) {
            return;
        }

        let curTime = time;
        changeIntoProgressing();
        clearTimer();
        setText(`${progressText}(${time}s)`);
        timer.current = window.setInterval(() => {
            curTime -= 1;
            if (curTime === 0) {
                clearTimer();
                changeIntoInit();
                setText(init);
            } else {
                setText(`${progressText}(${curTime}s)`);
            }
        }, 1000);
    }, []);

    return [text, trigger, running.current];
}
// 播放进度条组件
function PlayerControlBar() {
    const { isMainConnected, mainApi, setMetaData } = useWebSocketServices();
    const { t } = useTranslation('playControlBar');
    const [hmi] = useHmiStore();
    const recordInfo = useMemo<IRecordInfo & { playRecordStatus: any; disabled: boolean }>(
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
    const onEnd = () => {
        if (recordInfo.disabled) {
            return;
        }
        mainApi.ctrRecorderAction('pause', recordInfo.currTimeS);
    };

    // 开始播放record
    const onStart = () => {
        if (recordInfo.disabled) {
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
    };

    const [dump, triggerDump, dumpTimerStatus] = useTimeDown(t('dumpMsg', { path: '/tmp' }), 3);
    const [dumpStatus, setDumpStatus] = useState<EMUN_OPERATE_STATUS>(EMUN_OPERATE_STATUS.SUCCESS);
    const [reset, triggerReset, resetTimerStatus] = useTimeDown(t('resetMsg'), 3);
    const [resetStatus, setResetStatus] = useState<EMUN_OPERATE_STATUS>(EMUN_OPERATE_STATUS.SUCCESS);

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

    // 重置record
    const onReset = () => {
        if (resetStatus === EMUN_OPERATE_STATUS.PROGRESSING || resetTimerStatus === EMUN_TIMEDOWN_STATUS.PORGRESSING) {
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

    const popoverStatus = (operateStatus: EMUN_OPERATE_STATUS, timedownStatus: EMUN_TIMEDOWN_STATUS) => {
        if (timedownStatus === EMUN_TIMEDOWN_STATUS.INIT) {
            return '';
        }
        if (operateStatus === EMUN_OPERATE_STATUS.SUCCESS) {
            return classes['operate-success'];
        }
        if (operateStatus === EMUN_OPERATE_STATUS.FAILED) {
            return classes['operate-failed'];
        }
        return '';
    };
    const resetPopoverStatus = popoverStatus(resetStatus, resetTimerStatus);
    const dumpPopoverStatus = popoverStatus(dumpStatus, dumpTimerStatus);

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
            <Popover rootClassName={dumpPopoverStatus} placement='topRight' trigger='hover' content={dump}>
                <div onClick={onDownload} className='player-download-btn'>
                    <IconIcBottomRailDownload />
                    <span className='player-download-btn-text'>{t('dump')}</span>
                </div>
            </Popover>
            <Popover rootClassName={resetPopoverStatus} placement='topRight' trigger='hover' content={reset}>
                <div onClick={onReset} className='player-reset-btn'>
                    <IconIcBottomRailDelete />
                    <span className='player-download-reset-text'>{t('reset')}</span>
                </div>
            </Popover>
        </div>
    );
}

export default React.memo(PlayerControlBar);
