import React, { useRef, useState, useCallback, useEffect, useMemo } from 'react';
import { MosaicNode, MosaicParent } from 'react-mosaic-component';
import { message } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import { BusinessEventTypes, BusinessEventInfo } from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import { usePanelInfoStore } from '@dreamview/dreamview-core/src/store/PanelInfoStore';
import { useGetCurrentLayout } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { PanelType } from '@dreamview/dreamview-core/src/components/panels/type/Panel';

export enum EMUN_OPERATE_STATUS {
    SUCCESS,
    PROGRESSING,
    FAILED,
}
export enum PlayRecordStatus {
    RUNNING = 'RUNNING',
    PAUSED = 'PAUSED',
    CLOSED = 'CLOSED',
}

export enum PlayRTKRecordStatus {
    OK = 'OK',
    FATAL = 'FATAL',
    DISABLE = 'DISABLE',
}

export enum EMUN_TIMEDOWN_STATUS {
    INIT,
    PORGRESSING,
}

export const popoverStatus = (operateStatus: EMUN_OPERATE_STATUS, timedownStatus: EMUN_TIMEDOWN_STATUS) => {
    if (timedownStatus === EMUN_TIMEDOWN_STATUS.INIT) {
        return '';
    }
    if (operateStatus === EMUN_OPERATE_STATUS.SUCCESS) {
        return 'operate-success';
    }
    if (operateStatus === EMUN_OPERATE_STATUS.FAILED) {
        return 'operate-failed';
    }
    return '';
};

export function useTimeDown(
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

    useEffect(
        () => () => {
            clearTimer();
        },
        [],
    );

    return [text, trigger, running.current];
}

export function useKeyDownEvent(onKeyDownPlaycallback: () => void, onKeyDownStopcallback: () => void) {
    useEffect(() => {
        const handleKeyDown = (e: KeyboardEvent) => {
            if (e.ctrlKey && e.key === 'p') {
                onKeyDownPlaycallback();
            }
            if (e.ctrlKey && e.key === 's') {
                onKeyDownStopcallback();
            }
        };
        window.addEventListener('keydown', handleKeyDown);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
        };
    }, [onKeyDownPlaycallback, onKeyDownStopcallback]);
}

export function CountPanelAndCollectId(layout: MosaicNode<string>, panelname: string) {
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

export function useSendRouting(
    routingInfo: Record<string, BusinessEventInfo[BusinessEventTypes.SimControlRoute]['routeInfo']>,
    keyEvent?: boolean,
) {
    const layout = useGetCurrentLayout();
    const { t } = useTranslation('pncMonitor');
    const [panelInfoState] = usePanelInfoStore();
    const { mainApi, isMainConnected } = useWebSocketServices();
    const [hmi] = usePickHmiStore();

    const { panelVehicleVisCount, vehicleVisIds } = useMemo(
        () => CountPanelAndCollectId(layout, PanelType.VehicleViz),
        [layout],
    );

    const route = useMemo(() => {
        let errorMessage = '';
        let currentRoute = null;

        if (panelVehicleVisCount === 0) {
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
    }, [panelVehicleVisCount, panelInfoState, routingInfo, vehicleVisIds, t]);

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

    const startRoutingRequest = useCallback(
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
        if (!hmi.modules?.get('Planning')) {
            message({
                type: 'warning',
                content: t('simulationDisabled5'),
            });
        }
        const waypoint = [...route.currentRoute.wayPoint];
        const endPonit = waypoint.pop();
        const isCycle = !!route.currentRoute.cycleNumber;
        if (isCycle) {
            startCycle(endPonit, waypoint, route.currentRoute.cycleNumber);
        } else {
            startRoutingRequest(waypoint, endPonit);
        }
    }, [isMainConnected, route, startCycle, startRoutingRequest, hmi.modules, t]);

    const onEnd = useCallback(() => {
        if (!isMainConnected) {
            return;
        }
        if (route.errorMessage) {
            return;
        }
        mainApi.resetSimControl();
    }, [isMainConnected, route, mainApi]);

    useEffect(() => {
        if (!keyEvent) {
            return () => null;
        }
        const handleKeyDown = (e: KeyboardEvent) => {
            if (e.ctrlKey && e.key === 'p') {
                onStart();
            }
            if (e.ctrlKey && e.key === 's') {
                onEnd();
            }
        };
        window.addEventListener('keydown', handleKeyDown);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
        };
    }, [onStart, onEnd]);

    return useMemo(
        () => ({
            send: onStart,
            stop: onEnd,
            routingInfo: route,
        }),
        [onStart, route, onEnd],
    );
}
