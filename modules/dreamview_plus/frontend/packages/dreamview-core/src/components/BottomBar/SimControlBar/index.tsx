import React, { useCallback, useMemo } from 'react';
import { IconIcOverUsable, IconIcStopPlaying, IconIcNotApplicable, IconIcPlay } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import Popover from '@dreamview/dreamview-core/src/components/CustomPopover';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { usePickHmiStore, ENUM_DATARECORD_PRECESS_STATUS } from '@dreamview/dreamview-core/src/store/HmiStore';
import { MosaicNode, MosaicParent } from 'react-mosaic-component';
import { BusinessEventTypes, BusinessEventInfo } from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import { usePanelInfoStore } from '@dreamview/dreamview-core/src/store/PanelInfoStore';
import { usePanelLayoutStore } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { PanelType } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import useStyle from './useStyle';
import DumpBtn from '../Operate/Dump';
import ResetBtn from '../Operate/Reset';
import { useKeyDownEvent } from '../util';
import RecordBtn from '../Operate/Record';

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

function SimControlBar(props: ScenarioBarProps) {
    const { routingInfo } = props;
    const [panelInfoState] = usePanelInfoStore();
    const [{ layout }] = usePanelLayoutStore();
    const { t } = useTranslation('pncMonitor');
    const { mainApi, isMainConnected } = useWebSocketServices();
    const [hmi] = usePickHmiStore();
    const { classes, cx } = useStyle();

    const { panelVehicleVisCount, vehicleVisIds } = useMemo(
        () => CountPanelAndCollectId(layout, PanelType.VehicleViz),
        [layout],
    );

    const route = useMemo(() => {
        let errorMessage = '';
        let currentRoute = null;
        if (!hmi.moduleStatus?.get('Planning')) {
            errorMessage = t('simulationDisabled5');
            currentRoute = null;
        } else if (panelVehicleVisCount === 0) {
            errorMessage = t('simulationDisabled1');
            currentRoute = null;
        } else if (panelVehicleVisCount === 1) {
            const info = routingInfo[vehicleVisIds[0]];
            if (!info?.wayPoint?.length) {
                errorMessage = t('simulationDisabled2');
                currentRoute = null;
            } else {
                errorMessage = '';
                currentRoute = info;
            }
        } else {
            const activeVehicleVizId = vehicleVisIds.find((id) => panelInfoState.selectedPanelIds.has(id));
            if (activeVehicleVizId) {
                const info = routingInfo[activeVehicleVizId];
                if (!info?.wayPoint?.length) {
                    errorMessage = t('simulationDisabled3');
                    currentRoute = null;
                } else {
                    errorMessage = '';
                    currentRoute = info;
                }
            } else {
                errorMessage = t('simulationDisabled4');
                currentRoute = null;
            }
        }

        return {
            errorMessage,
            currentRoute,
        };
    }, [hmi.moduleStatus, panelVehicleVisCount, panelInfoState, routingInfo, vehicleVisIds, t]);

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

    const startSimControl = useCallback(
        (waypoint: BusinessEventInfo['RoutePoint'][], end: BusinessEventInfo['RoutePoint']) => {
            if (!mainApi) {
                return;
            }
            mainApi.sendRoutingRequest({
                waypoint,
                end,
            });
        },
        [mainApi],
    );

    const onStart = useCallback(() => {
        if (!isMainConnected) {
            return;
        }
        if (route.errorMessage) {
            return;
        }
        const waypoint = [...route.currentRoute.wayPoint];
        const endPonit = waypoint.pop();
        const isCycle = !!route.currentRoute.cycleNumber;
        if (isCycle) {
            startCycle(endPonit, waypoint, route.currentRoute.cycleNumber);
        } else {
            startSimControl(waypoint, endPonit);
        }
    }, [isMainConnected, route, startCycle, startSimControl]);

    const onEnd = useCallback(() => {
        if (!isMainConnected) {
            return;
        }
        if (route.errorMessage) {
            return;
        }
        mainApi.resetSimControl();
    }, [isMainConnected, route, mainApi]);

    const playIc = <IconIcNotApplicable onClick={onStart} className='ic-play-btn' />;

    const popoverPlayIc = route.errorMessage ? (
        <Popover placement='topLeft' trigger='hover' content={route.errorMessage}>
            {playIc}
        </Popover>
    ) : (
        playIc
    );

    const stopIc = <IconIcOverUsable className='ic-stop-btn' onClick={onEnd} />;

    useKeyDownEvent(onStart, onEnd);

    return (
        <div className={cx(classes['record-controlbar-container'], { [classes.disabled]: route.errorMessage })}>
            <div id='guide-simulation-record' className='ic-play-container'>
                {popoverPlayIc}
                {stopIc}
            </div>
            <div className={classes['flex-center']}>
                <RecordBtn />
                <DumpBtn disabled />
                <ResetBtn disabled />
            </div>
        </div>
    );
}

export default React.memo(SimControlBar);
