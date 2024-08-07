import React, { useCallback, useMemo } from 'react';
import { useTranslation } from 'react-i18next';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { usePickHmiStore, SIM_CONTROL_STATUS } from '@dreamview/dreamview-core/src/store/HmiStore';
import { usePanelInfoStore } from '@dreamview/dreamview-core/src/store/PanelInfoStore';
import { useGetCurrentLayout } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { PanelType } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import { BusinessEventTypes, BusinessEventInfo } from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import { message } from '@dreamview/dreamview-ui';
import useStyle from './useStyle';
import DumpBtn from '../Operate/Dump';
import ResetBtn from '../Operate/Reset';
import RecordBtn from '../Operate/Record';
import { useKeyDownEvent, CountPanelAndCollectId } from '../util';
import { DynamicEffectButtonMemo, DynamicEffectButtonStatus } from '../DynamicEffectButton';

interface ScenarioBarProps {
    routingInfo: Record<string, BusinessEventInfo[BusinessEventTypes.SimControlRoute]['routeInfo']>;
}

function ScenarioBar(props: ScenarioBarProps) {
    const { routingInfo } = props;
    const [panelInfoState] = usePanelInfoStore();
    const layout = useGetCurrentLayout();
    const { t } = useTranslation('pncMonitor');
    const { mainApi, isMainConnected, otherApi } = useWebSocketServices();
    const [hmi] = usePickHmiStore();
    const { t: tBottomBar } = useTranslation('bottomBar');

    const { panelVehicleVisCount, vehicleVisIds } = useMemo(
        () => CountPanelAndCollectId(layout, PanelType.VehicleViz),
        [layout],
    );

    const getRouteInfo = useCallback(() => {
        if (panelVehicleVisCount === 0) {
            return null;
        }
        if (panelVehicleVisCount === 1) {
            return routingInfo[vehicleVisIds[0]];
        }
        // panelVehicleVisCount >= 2
        const activeVehicleVizId = vehicleVisIds.find((id) => panelInfoState.selectedPanelIds.has(id));
        if (activeVehicleVizId) {
            return routingInfo[activeVehicleVizId];
        }

        return null;
    }, [panelVehicleVisCount, vehicleVisIds, routingInfo, panelInfoState]);

    const { classes, cx } = useStyle();

    const startCycle = useCallback(
        (end: BusinessEventInfo['RoutePoint'], waypoint: BusinessEventInfo['RoutePoint'][], cycleNumber: number) => {
            if (!mainApi) {
                return;
            }
            mainApi.startCycle({
                cycleNumber,
                end,
                waypoint,
            });
        },
        [mainApi],
    );

    const errormsg = useMemo(() => {
        if (!hmi.currentScenarioId) {
            return t('scenarioSimulationDisabled1');
        }
        return '';
    }, [hmi.currentScenarioId, t]);

    const startScenario = useCallback(
        (
            fromScenario: boolean,
            end?: BusinessEventInfo['RoutePoint'],
            waypoint?: BusinessEventInfo['RoutePoint'][],
        ) => {
            otherApi
                ?.startScenario({
                    fromScenario,
                    end,
                    waypoint,
                })
                .catch((err) => {
                    console.error(err);
                    message({
                        type: 'error',
                        content: t('scenarioSimulationFailed'),
                    });
                });
        },
        [mainApi],
    );

    const onStart = useCallback(() => {
        if (errormsg) {
            return;
        }
        if (!isMainConnected) {
            return;
        }
        if (!hmi.modules?.get('Planning')) {
            message({
                type: 'warning',
                content: t('scenarioSimulationDisabled2'),
            });
        }
        const info = getRouteInfo();
        if (info) {
            const waypoint = [...(info.wayPoint || [])];
            const endPonit = waypoint.pop();
            const isCycle = !!info.cycleNumber;
            if (isCycle) {
                startCycle(endPonit, waypoint, info.cycleNumber);
            } else {
                startScenario(!endPonit, endPonit, waypoint);
            }
        } else {
            startScenario(true);
        }
    }, [hmi.modules, errormsg, isMainConnected, getRouteInfo, startCycle, startScenario, mainApi, t]);

    const onEnd = useCallback(() => {
        if (errormsg) {
            return;
        }
        if (isMainConnected) {
            otherApi.stopScenario();
        }
    }, [errormsg, isMainConnected, mainApi]);

    const onReset = useCallback(() => {
        if (errormsg) {
            return;
        }
        if (isMainConnected) {
            otherApi.resetScenario();
        }
    }, [errormsg, isMainConnected]);

    useKeyDownEvent(onStart, onEnd);

    const buttonStatus = useMemo(() => {
        if (errormsg) {
            return DynamicEffectButtonStatus.DISABLE;
        }
        return hmi.simOtherComponents?.ScenarioSimulation?.message === SIM_CONTROL_STATUS.OK
            ? DynamicEffectButtonStatus.STOP
            : DynamicEffectButtonStatus.START;
    }, [errormsg, hmi.simOtherComponents]);

    const behavior = {
        [DynamicEffectButtonStatus.DISABLE]: {
            disabledMsg: errormsg,
            text: tBottomBar('Start'),
        },
        [DynamicEffectButtonStatus.START]: {
            text: tBottomBar('Start'),
            clickHandler: onStart,
        },
        [DynamicEffectButtonStatus.STOP]: {
            text: tBottomBar('Stop'),
            clickHandler: onEnd,
        },
    };

    const resetBtnStatus = (() => {
        if (errormsg) {
            return DynamicEffectButtonStatus.DISABLE;
        }
        return DynamicEffectButtonStatus.STOP;
    })();

    return (
        <div className={cx(classes['record-controlbar-container'], { [classes.disabled]: errormsg })}>
            <div id='guide-simulation-record' className='ic-play-container'>
                <DynamicEffectButtonMemo behavior={behavior} status={buttonStatus} />
                &nbsp;&nbsp;&nbsp;&nbsp;
                <DynamicEffectButtonMemo
                    behavior={{
                        [DynamicEffectButtonStatus.STOP]: {
                            text: tBottomBar('Reset'),
                            clickHandler: onReset,
                        },
                        [DynamicEffectButtonStatus.DISABLE]: {
                            disabledMsg: errormsg,
                            text: tBottomBar('Reset'),
                        },
                    }}
                    status={resetBtnStatus}
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

export default React.memo(ScenarioBar);
