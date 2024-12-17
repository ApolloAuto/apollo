import React, { useCallback, useEffect, useMemo } from 'react';
import Logger from '@dreamview/log';
import { Popover, IconPark } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { useHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import { BusinessEventTypes, BusinessEventInfo } from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import useStyle from './useStyle';
import { PlayRecordStatus, useKeyDownEvent, useSendRouting } from '../util';
import Progress from './Progress';
import DumpBtn from '../Operate/Dump';
import ResetBtn from '../Operate/Reset';
import Record from '../Operate/Record';

const logger = Logger.getInstance('PlayerControlBar');

function formatTs(v: number) {
    if (!v) return 0;

    return Math.floor(Math.round(v * 100)) / 100;
}

interface PlayerControlBarProps {
    routingInfo: Record<string, BusinessEventInfo[BusinessEventTypes.SimControlRoute]['routeInfo']>;
}
// 播放进度条组件
function PlayerControlBar(props: PlayerControlBarProps) {
    const { isMainConnected, mainApi } = useWebSocketServices();
    const { t } = useTranslation('bottomBar');
    const [hmi] = useHmiStore();
    const routingManager = useSendRouting(props.routingInfo, false);

    const recordInfo = useMemo(
        () => ({
            totalTimeS: formatTs(hmi?.records[hmi?.currentRecordStatus?.currentRecordId]?.totalTimeS || 0),
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
        if (!isMainConnected) return;
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
    }, [mainApi, isMainConnected, recordInfo.disabled, recordInfo.playRecordStatus, recordInfo.currTimeS]);

    const playIc = isInPlaying ? (
        <IconPark name='IcPlay' disabled={recordInfo.disabled} onClick={onEnd} className='ic-play-btn' />
    ) : (
        <IconPark name='IcStopPlaying' onClick={onStart} className='ic-play-btn' />
    );

    const popoverPlayIc = recordInfo.disabled ? (
        <Popover placement='topLeft' trigger='hover' content={t('recordMsg')}>
            {playIc}
        </Popover>
    ) : (
        playIc
    );

    const popoverRoutingIc =
        recordInfo.disabled || routingManager.routingInfo.errorMessage ? (
            <Popover
                placement='topLeft'
                trigger='hover'
                content={recordInfo.disabled ? t('recordMsg') : routingManager.routingInfo.errorMessage}
            >
                <IconPark name='IcSissue' className='ic-routing-btn' />
            </Popover>
        ) : (
            <Popover placement='topLeft' trigger='hover' content={t('routing')}>
                <IconPark name='IcSissue' onClick={routingManager.send} className='ic-routing-btn' />
            </Popover>
        );

    useKeyDownEvent(onStart, onEnd);

    return (
        <div className={cx(classes['player-controlbar-container'], { [classes.disabled]: recordInfo.disabled })}>
            <span
                className={cx({
                    [classes.disabled]: !!routingManager.routingInfo.errorMessage,
                })}
                id='guide-simulation-record'
            >
                {popoverRoutingIc}
            </span>
            <span id='guide-simulation-record'>{popoverPlayIc}</span>
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
            <DumpBtn disabled={false} />
            <ResetBtn disabled={false} />
        </div>
    );
}

export default React.memo(PlayerControlBar);
