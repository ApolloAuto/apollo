import React, { useCallback, useEffect, useRef, useState } from 'react';
import { IconPark } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { BusinessEventTypes, BusinessEventInfo } from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import throttle from 'lodash/throttle';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import useStyle from './useStyle';
import DumpBtn from '../Operate/Dump';
import ResetBtn from '../Operate/Reset';
import { useSendRouting } from '../util';
import RecordBtn from '../Operate/Record';
import { DynamicEffectButtonMemo, DynamicEffectButtonStatus } from '../DynamicEffectButton';

interface AutoDriveBarProps {
    routingInfo: Record<string, BusinessEventInfo[BusinessEventTypes.SimControlRoute]['routeInfo']>;
}

function AutoDriveBar(props: AutoDriveBarProps) {
    const { routingInfo } = props;
    const { t } = useTranslation('bottomBar');
    const { mainApi, streamApi } = useWebSocketServices();
    const { classes, cx } = useStyle();
    const routingManager = useSendRouting(routingInfo);
    const handleStartAutoClick = useCallback(
        throttle(() => {
            mainApi.startAutoDrive();
        }, 300),
        [],
    );

    const [isInAutoDriving, setIsInAutoDriving] = useState(false);
    const StartAutoButtonStatus = isInAutoDriving ? DynamicEffectButtonStatus.RUNNING : DynamicEffectButtonStatus.START;

    const unSubsctibe = useRef<any>();

    useEffect(() => {
        unSubsctibe.current = streamApi.subscribeToData(StreamDataNames.SIM_WORLD).subscribe((data: any) => {
            setIsInAutoDriving(data?.autoDrivingCar?.disengageType === 'DISENGAGE_NONE');
        });
        return () => {
            unSubsctibe.current.unsubscribe();
        };
    }, []);

    const routingButtonStatus = routingManager.routingInfo.errorMessage
        ? DynamicEffectButtonStatus.DISABLE
        : DynamicEffectButtonStatus.START;

    return (
        <div className={cx(classes['record-controlbar-container'])}>
            <div id='guide-simulation-record' className='ic-play-container'>
                <DynamicEffectButtonMemo
                    behavior={{
                        [DynamicEffectButtonStatus.RUNNING]: {
                            text: t('Running'),
                            icon: <IconPark name='IcAuto' />,
                        },
                        [DynamicEffectButtonStatus.START]: {
                            text: t('StartAutoDraive'),
                            icon: <IconPark name='IcAuto' />,
                            clickHandler: handleStartAutoClick,
                        },
                    }}
                    status={StartAutoButtonStatus}
                />
                &nbsp;&nbsp;&nbsp;&nbsp;
                <DynamicEffectButtonMemo
                    behavior={{
                        [DynamicEffectButtonStatus.START]: {
                            text: t('sendRouting'),
                            icon: <IconPark name='IcSissue' />,
                            clickHandler: routingManager.send,
                        },
                        [DynamicEffectButtonStatus.DISABLE]: {
                            text: t('sendRouting'),
                            icon: <IconPark name='IcSissue' />,
                            clickHandler: routingManager.send,
                            disabledMsg: routingManager.routingInfo.errorMessage,
                        },
                    }}
                    status={routingButtonStatus}
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

export default React.memo(AutoDriveBar);
