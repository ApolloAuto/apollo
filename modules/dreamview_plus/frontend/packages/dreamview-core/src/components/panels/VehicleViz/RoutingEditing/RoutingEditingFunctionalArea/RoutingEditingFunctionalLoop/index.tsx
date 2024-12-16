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
