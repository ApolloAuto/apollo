import React from 'react';
import { IconPark } from '@dreamview/dreamview-ui';
import { BusinessEventTypes, BusinessEventInfo } from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import { useTranslation } from 'react-i18next';
import { useSendRouting } from '../util';
import useStyle from './useStyle';
import DumpBtn from '../Operate/Dump';
import ResetBtn from '../Operate/Reset';
import RecordBtn from '../Operate/Record';
import { DynamicEffectButtonMemo, DynamicEffectButtonStatus } from '../DynamicEffectButton';

interface ScenarioBarProps {
    routingInfo: Record<string, BusinessEventInfo[BusinessEventTypes.SimControlRoute]['routeInfo']>;
}

function SimControlBar(props: ScenarioBarProps) {
    const { routingInfo } = props;
    const { classes, cx } = useStyle();
    const { t: tBottomBar } = useTranslation('bottomBar');

    const routing = useSendRouting(routingInfo);

    const StartSimControlBtnStatus = (() => {
        if (routing.routingInfo.errorMessage) {
            return DynamicEffectButtonStatus.DISABLE;
        }
        return DynamicEffectButtonStatus.START;
    })();

    const StopBtnStatus = (() => {
        if (routing.routingInfo.errorMessage) {
            return DynamicEffectButtonStatus.DISABLE;
        }
        return DynamicEffectButtonStatus.STOP;
    })();

    return (
        <div
            className={cx(classes['record-controlbar-container'], {
                [classes.disabled]: !!routing.routingInfo.errorMessage,
            })}
        >
            <div id='guide-simulation-record' className='ic-play-container'>
                <DynamicEffectButtonMemo
                    behavior={{
                        [DynamicEffectButtonStatus.DISABLE]: {
                            text: tBottomBar('Start'),
                            disabledMsg: routing.routingInfo.errorMessage,
                        },
                        [DynamicEffectButtonStatus.START]: {
                            text: tBottomBar('Start'),
                            clickHandler: routing.send,
                        },
                    }}
                    status={StartSimControlBtnStatus}
                />
                &nbsp;&nbsp;&nbsp;&nbsp;
                <DynamicEffectButtonMemo
                    behavior={{
                        [DynamicEffectButtonStatus.STOP]: {
                            text: tBottomBar('Stop'),
                            clickHandler: routing.stop,
                        },
                        [DynamicEffectButtonStatus.DISABLE]: {
                            text: tBottomBar('Stop'),
                            icon: <IconPark name='IcOverUsable' />,
                            disabledMsg: routing.routingInfo.errorMessage,
                        },
                    }}
                    status={StopBtnStatus}
                />
            </div>
            <div className={classes['flex-center']}>
                <RecordBtn />
                <DumpBtn disabled={false} />
                <ResetBtn disabled={false} />
            </div>
        </div>
    );
}

export default React.memo(SimControlBar);
