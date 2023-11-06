import React, { useCallback, useEffect, useMemo, useState } from 'react';
import { IconIcNotApplicable, IconIcOverUsable } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import Popover from '@dreamview/dreamview-core/src/components/CustomPopover';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { usePickHmiStore, SIM_CONTROL_STATUS } from '@dreamview/dreamview-core/src/store/HmiStore';
import { usePanelInfoStore } from '@dreamview/dreamview-core/src/store/PanelInfoStore';
import { usePanelLayoutStore } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { MosaicNode, MosaicParent } from 'react-mosaic-component';
import { PanelType } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import { BusinessEventTypes, BusinessEventInfo } from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import useStyle from './useStyle';
import DumpBtn from '../Operate/Dump';
import ResetBtn from '../Operate/Reset';
import RecordBtn from '../Operate/Record';
import { useKeyDownEvent } from '../util';

function CountPanelAndCollectId(layout: MosaicNode<string>, panelname: string) {
    const vehicleVisIds: Array<string> = [];
    // panelId is like 'panelName!hash'
    const hasChildren = (node: MosaicNode<string>) => typeof node !== 'string';
    const isPanelMatch = (panelId: string) => panelId.match(panelname);
    if (!layout) {
        return {
            panelVehicleVisCount: 0,
            vehicleVisIds: [],
        };
    }

    if (typeof layout === 'string') {
        return {
            panelVehicleVisCount: isPanelMatch(layout) ? 1 : 0,
            vehicleVisIds: [layout],
        };
    }

    let panelVehicleVisCount = 0;
    function reduce(node: MosaicNode<string>) {
        if (hasChildren(node)) {
            reduce((node as MosaicParent<string>).first);
            reduce((node as MosaicParent<string>).second);
            return;
        }
        const isMatch = isPanelMatch(node as string);
        if (isMatch) {
            panelVehicleVisCount += 1;
            vehicleVisIds.push(node as string);
        }
    }
    reduce(layout);

    return {
        panelVehicleVisCount,
        vehicleVisIds,
    };
}

interface ScenarioBarProps {
    routingInfo: Record<string, BusinessEventInfo[BusinessEventTypes.SimControlRoute]['routeInfo']>;
}

function ScenarioBar(props: ScenarioBarProps) {
    const { routingInfo } = props;
    const [panelInfoState] = usePanelInfoStore();
    const [{ layout }] = usePanelLayoutStore();
    const { t } = useTranslation('pncMonitor');
    const { mainApi, isMainConnected } = useWebSocketServices();
    const [hmi] = usePickHmiStore();

    const isInScenarioControl = hmi.otherComponents?.ScenarioSimulation?.status === SIM_CONTROL_STATUS.OK;

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
        if (!hmi.moduleStatus?.get('Planning')) {
            return t('scenarioSimulationDisabled2');
        }
        return '';
    }, [hmi.moduleStatus, hmi.currentScenarioId, t]);

    const startScenario = useCallback(
        (fromScenario: boolean, end: BusinessEventInfo['RoutePoint'], waypoint: BusinessEventInfo['RoutePoint'][]) => {
            if (!mainApi) {
                return;
            }
            mainApi.startScenario({
                fromScenario,
                end,
                waypoint,
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
            mainApi.startScenario({
                fromScenario: true,
            });
        }
    }, [errormsg, isMainConnected, getRouteInfo, startCycle, startScenario, mainApi]);

    const onEnd = useCallback(() => {
        if (errormsg) {
            return;
        }
        if (isMainConnected) {
            mainApi.stopScenario();
        }
    }, [errormsg, isMainConnected, mainApi]);

    const playIc = isInScenarioControl ? (
        <IconIcOverUsable onClick={onEnd} className='ic-stop-btn' />
    ) : (
        <IconIcNotApplicable onClick={onStart} className='ic-play-btn' />
    );

    const popoverPlayIc = errormsg ? (
        <Popover placement='topLeft' trigger='hover' content={errormsg}>
            {playIc}
        </Popover>
    ) : (
        playIc
    );

    useKeyDownEvent(onStart, onEnd);

    return (
        <div className={cx(classes['record-controlbar-container'], { [classes.disabled]: errormsg })}>
            <div id='guide-simulation-record' className='ic-play-container'>
                {popoverPlayIc}
            </div>
            <div className={classes['flex-center']}>
                <RecordBtn />
                <DumpBtn disabled />
                <ResetBtn disabled />
            </div>
        </div>
    );
}

export default React.memo(ScenarioBar);
