import React, { useEffect, useState } from 'react';
import { IconPark, Popover, Switch, InputNumber, message } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';
import { useVizStore } from '../../../VizStore';

interface RoutingEditingFunctionalLoopProps {}

export default function RoutingEditingFunctionalLoop(props: RoutingEditingFunctionalLoopProps) {
    const { classes, cx } = useStyle();

    const [{ routeManager }] = useVizStore();

    const { t } = useTranslation('routeEditing');

    const routeManagerMix = routeManager.currentRouteMix;

    const currentRouteMix = routeManagerMix.getCurrentRouteMix();

    const [checked, setChecked] = useState(currentRouteMix.currentRouteLoop.currentRouteLoopState);

    const handleSwitch = (switchVal: boolean) => {
        if (switchVal) {
            routeManagerMix.setCurrentRouteMix({
                currentRouteLoop: { currentRouteLoopState: true, currentRouteLoopTimes: 1 },
            });
        } else {
            routeManagerMix.setCurrentRouteMix({
                currentRouteLoop: { currentRouteLoopState: false },
            });
        }
        setChecked(switchVal);
    };

    const handleInput = (inputVal: any) => {
        routeManagerMix.setCurrentRouteMix({
            currentRouteLoop: { currentRouteLoopState: true, currentRouteLoopTimes: inputVal },
        });
    };

    // useEffect(() => {
    //     const wayCount = routingEditor.pathwayMarker.positionsCount;
    //     if (wayCount > 0) {
    //         if (isMainConnected) {
    //             mainApi.getStartPoint().then((res) => {
    //                 const startPoint = { x: res.x, y: res.y, heading: res?.heading };
    //                 const endPoint = routingEditor.pathwayMarker.lastPosition;
    //                 mainApi.checkCycleRouting({ start: startPoint, end: endPoint }).then((cycle) => {
    //                     if (cycle.isCycle) {
    //                         setLoopDisable(false);
    //                         if (routeManagerMix.getCurrentRouteMix().currentRouteLoop.currentRouteLoopState) {
    //                             setChecked(true);
    //                             setInputDisplay(true);
    //                         }
    //                     } else {
    //                         const currentRouteMixValue = { currentRouteLoop: { currentRouteLoopState: false } };
    //                         routeManagerMix.setCurrentRouteMix(currentRouteMixValue);
    //                         setChecked(false);
    //                         setLoopDisable(true);
    //                         message({
    //                             type: 'error',
    //                             content: t('NoLoopMessage'),
    //                         });
    //                     }
    //                 });
    //             });
    //         }
    //     } else {
    //         message({ type: 'error', content: t('NoWayPointMessage') });
    //     }
    // }, [isMainConnected]);

    return (
        <div className={classes['functional-loop-con']}>
            <div className={classes['functional-loop-switch']}>
                <div>{`${t('loopRouting')}:`}</div>
                <div>
                    <Switch onChange={handleSwitch} checked={checked} />
                </div>
                <Popover
                    content={t('loopRoutingHelp')}
                    trigger='hover'
                    placement='top'
                    rootClassName={classes['functional-loop-switch-help']}
                >
                    <div className={classes['functional-loop-switch-remind']}>
                        <IconPark name='IcHelpNormal' />
                    </div>
                </Popover>
            </div>
            {checked && (
                <div className={classes['functional-loop-input']}>
                    <div className={classes['functional-loop-input-text']}>{`${t('setLooptimes')}:`}</div>
                    <div>
                        <InputNumber
                            min={1}
                            max={10}
                            precision={0}
                            onChange={handleInput}
                            defaultValue={routeManagerMix.getCurrentRouteMix().currentRouteLoop.currentRouteLoopTimes}
                        />
                    </div>
                </div>
            )}
        </div>
    );
}
