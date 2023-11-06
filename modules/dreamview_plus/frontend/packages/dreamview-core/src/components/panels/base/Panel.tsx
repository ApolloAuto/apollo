import React, { useCallback, useContext, useEffect, useMemo, useRef, useState } from 'react';
import Logger from '@dreamview/log';
import {
    MosaicContext,
    MosaicDirection,
    MosaicWindowActions,
    MosaicWindowContext,
    getAndAssertNodeAtPathExists,
} from 'react-mosaic-component';
import { useResizeDetector } from 'react-resize-detector';
import { OnResizeCallback } from 'react-resize-detector/build/types/types';
import shortUUID from 'short-uuid';
import { Observable, Subscription, combineLatest, of, withLatestFrom, zip } from 'rxjs';
import { isEmpty } from 'lodash';
import { useTranslation } from 'react-i18next';
import { FullScreenHookConfig, InitSubscriptionMap, PanelContext, PanelMetaData } from './store/PanelStore';
import { usePanelCatalogContext } from '../../../store/PanelCatalogStore';
import { genereatePanelId } from '../../../util/layout';
import { noop } from '../../../util/similarFunctions';
import useWebSocketServices from '../../../services/hooks/useWebSocketServices';
import { PanelRoot } from './PanelRoot';
import useUpdateChannel from '../../../hooks/useUpdateChannel';
import EmptyPlaceHolder from './EmptyPlaceHolder';
import CountedSubject from '../../../util/CountedSubject';
import { SubscribeInfo } from '../type/RenderToolBar';
import useNotifyInitialChanel from '../../../hooks/useNotifyInitialChanel';
import useGetUpdateChannel from '../../../hooks/useGetUpdateChannel';
import { usePanelInfoStore } from '../../../store/PanelInfoStore';
import { addSelectedPanelId, deleteSelectedPanelId } from '../../../store/PanelInfoStore/actions';
import KeyListener, { KeyHandlers } from './KeyListener';
import { hooksManager } from '../../../util/HooksManager';
import { usePanelTileContext } from '../../../store/PanelInnerStore/PanelTileStore';

export type PanelProps = {
    PanelComponent: React.ComponentType & { displayName: string };
    panelId: string;
    placeHolder?: React.ReactNode;
    subscribeInfo?: SubscribeInfo[];
    panelMetaData?: PanelMetaData;
};

export default function Panel(panelProps: PanelProps) {
    const { PanelComponent, panelId, placeHolder, subscribeInfo, panelMetaData } = panelProps;
    function PanelWrapper(props: any) {
        const { isMainConnected, streamApi, metadata } = useWebSocketServices();
        const { panelCatalog } = usePanelCatalogContext();
        const { mosaicActions } = useContext(MosaicContext);
        const { mosaicWindowActions }: { mosaicWindowActions: MosaicWindowActions } = useContext(MosaicWindowContext);
        const [panelInfoState, panelInfoDispatch] = usePanelInfoStore();
        const { t } = useTranslation('panels');
        const [_panelMetaData, setPanelMetaData] = useState(panelMetaData);
        const { enterFullScreen, exitFullScreen, fullScreenFnObj } = usePanelTileContext();
        const [hasSubscribed, setHasSubscribed] = useState(false);
        const [hasData, setHasData] = useState(false);
        const [combinedData, setCombinedData] = useState<Record<string, any>>({});
        const connectedSubjectsRef = useRef<
            {
                name: string;
                connectedSubject: CountedSubject<unknown>;
            }[]
        >([]);
        const subscriptionsRef = useRef<
            {
                name: string;
                subscription: Subscription;
            }[]
        >([]);

        const initSubscriptionMapRef = useRef<InitSubscriptionMap>();
        const resizeCallBackRef = useRef<OnResizeCallback>();
        const resizeCallBackArrRef = useRef<OnResizeCallback[]>([]);
        const [keyDownHandlers, _setKeyDownHandlers] = useState<KeyHandlers[]>([
            {
                keys: ['escape'],
                handler: (e) => {
                    if (fullScreenFnObj && fullScreenFnObj?.exitFullScreen) {
                        fullScreenFnObj?.exitFullScreen();
                    }
                },
            },
        ]);
        const [keyUpHandlers, _setKeyUpHandlers] = useState<KeyHandlers[]>([]);
        const updateChannel = useGetUpdateChannel(panelId);

        const panel = useMemo(() => panelCatalog.get(panelId), [panelId, panelCatalog]);
        const isSelected = useMemo(() => panelInfoState?.selectedPanelIds?.has(panelId) ?? false, [panelInfoState]);

        const initSubscription = useMemo(
            () => (val: InitSubscriptionMap) => {
                initSubscriptionMapRef.current = val;
            },
            [],
        );

        const logger = useMemo(() => Logger.getInstance(panelId), [panelId]);

        const { ref: panelRootRef } = useResizeDetector({
            onResize: (width: number, height: number) => {
                if (resizeCallBackRef.current) {
                    resizeCallBackRef.current(width, height);
                }
                resizeCallBackArrRef.current.forEach((cb) => {
                    try {
                        cb(width, height);
                    } catch (err) {
                        console.log(err);
                    }
                });
            },
        });

        const onPanelResize = useCallback((onResize: OnResizeCallback) => {
            resizeCallBackRef.current = onResize;
            resizeCallBackArrRef.current.push(onResize);
        }, []);

        const splitPanel = useCallback(
            (direction: MosaicDirection) => {
                const root = mosaicActions.getRoot();
                const ownPath = mosaicWindowActions.getPath();
                const newId = genereatePanelId(panel?.type);
                mosaicActions.replaceWith(ownPath, {
                    direction,
                    second: newId,
                    first: getAndAssertNodeAtPathExists(root, ownPath),
                });
            },
            [mosaicWindowActions, mosaicActions, panel?.type],
        );

        const closePanel = useCallback(() => {
            const ownPath = mosaicWindowActions.getPath();
            mosaicActions.remove(ownPath);
        }, [mosaicActions, mosaicWindowActions]);

        const updateMetaData = useCallback(
            () => (newMetaData: PanelMetaData) => {
                setPanelMetaData({
                    ...newMetaData,
                    panelId,
                });
            },
            [panelId],
        );

        const onPanleClick: React.MouseEventHandler<HTMLDivElement> = useCallback(
            (event) => {
                if (!isSelected) {
                    panelInfoDispatch(addSelectedPanelId(panelId));
                }
            },
            [isSelected, panelInfoDispatch],
        );

        const checkDataEmpty = useCallback(
            (val: unknown) => {
                if (!hasData && val && !isEmpty(val)) {
                    setHasData(true);
                }
            },
            [hasData],
        );

        const handleSubscribe = useCallback(
            (newChannelInfo: SubscribeInfo) => {
                let newConnectedSubj: CountedSubject<unknown>;
                if (newChannelInfo?.needChannel) {
                    if (newChannelInfo?.name && newChannelInfo?.channel) {
                        if (newChannelInfo?.channel === 'default') {
                            newConnectedSubj = streamApi.subscribeToDataWithChannelFuzzy(newChannelInfo?.name);
                        } else {
                            newConnectedSubj = streamApi.subscribeToDataWithChannel(
                                newChannelInfo?.name,
                                newChannelInfo?.channel,
                            );
                        }
                    }
                } else {
                    newConnectedSubj = streamApi.subscribeToData(newChannelInfo?.name);
                }

                return newConnectedSubj;
            },
            [streamApi],
        );

        const defaultConsumer = useCallback((data: unknown) => {}, []);

        const addChannel = useCallback(
            (newChannelInfo: SubscribeInfo) => {
                if (isMainConnected) {
                    const curSubjects = connectedSubjectsRef.current;
                    const targetSubjectIndex = curSubjects.findIndex(
                        (curSubject) => curSubject.name === newChannelInfo.name,
                    );

                    if (targetSubjectIndex === -1) {
                        // 不存在 => 新增订阅
                        const newConnectedSubj: CountedSubject<unknown> = handleSubscribe(newChannelInfo);
                        if (newConnectedSubj) {
                            const subscription = newConnectedSubj.subscribe((val) => {
                                checkDataEmpty(val);

                                if (initSubscriptionMapRef.current && !isEmpty(initSubscriptionMapRef.current)) {
                                    const consumer =
                                        initSubscriptionMapRef.current[newChannelInfo.name]?.consumer ??
                                        defaultConsumer;
                                    consumer(val);
                                }
                            });

                            curSubjects.push({
                                name: newChannelInfo.name,
                                connectedSubject: newConnectedSubj,
                            });

                            subscriptionsRef.current.push({
                                name: newChannelInfo.name,
                                subscription,
                            });
                        }
                    } else {
                        // 存在 => 重新订阅
                        const subscriptionIndex = subscriptionsRef.current.findIndex(
                            (subscription) => subscription.name === newChannelInfo.name,
                        );
                        const curSubscription = subscriptionsRef.current[subscriptionIndex];

                        curSubscription.subscription.unsubscribe();

                        const newConnectedSubj: CountedSubject<unknown> = handleSubscribe(newChannelInfo);
                        if (newConnectedSubj) {
                            const newSubscription = newConnectedSubj.subscribe((val) => {
                                checkDataEmpty(val);

                                if (initSubscriptionMapRef.current && !isEmpty(initSubscriptionMapRef.current)) {
                                    const consumer =
                                        initSubscriptionMapRef.current[newChannelInfo.name]?.consumer ??
                                        defaultConsumer;
                                    consumer(val);
                                }
                            });

                            // 2.2替换掉connectedSubjectsRef.current数组中对应的subject
                            curSubjects[targetSubjectIndex] = {
                                name: newChannelInfo.name,
                                connectedSubject: newConnectedSubj,
                            };

                            subscriptionsRef.current[subscriptionIndex] = {
                                name: newChannelInfo.name,
                                subscription: newSubscription,
                            };
                        }
                    }
                }
            },
            [isMainConnected],
        );

        const closeSubcription = useCallback(
            (name: string) => {
                const targetSubjectIndex = connectedSubjectsRef.current.findIndex(
                    (curSubject) => curSubject.name === name,
                );

                if (targetSubjectIndex !== -1) {
                    const subscriptionIndex = subscriptionsRef.current.findIndex(
                        (subscription) => subscription.name === name,
                    );
                    const curSubscription = subscriptionsRef.current[subscriptionIndex];

                    curSubscription.subscription.unsubscribe();

                    subscriptionsRef.current = subscriptionsRef.current.filter(
                        (subscriptionInfo) => subscriptionInfo.name !== name,
                    );
                    connectedSubjectsRef.current = connectedSubjectsRef.current.filter(
                        (connectedSubjectInfo) => connectedSubjectInfo.name !== name,
                    );
                }
            },
            [isMainConnected],
        );

        const setKeyDownHandlers = useCallback((handlers: KeyHandlers[]) => {
            _setKeyDownHandlers((prevHandlers) => [...prevHandlers, ...handlers]);
        }, []);

        const setKeyUpHandlers = useCallback((handlers: KeyHandlers[]) => {
            _setKeyUpHandlers((prevHandlers) => [...prevHandlers, ...handlers]);
        }, []);

        const registerFullScreenHooks = useCallback((hookConfig: FullScreenHookConfig) => {
            hooksManager.addHook(panelId, hookConfig);
        }, []);

        const isPlaceHolderDisplay = useMemo(
            () => !(!subscribeInfo || (hasSubscribed && hasData)),
            // 调试使用
            // () => false,
            [hasSubscribed, hasData],
        );

        const NoDataPlaceHolder = useMemo(() => {
            if (placeHolder) {
                return placeHolder;
            }
            return (
                <EmptyPlaceHolder
                    style={{
                        display: !isPlaceHolderDisplay ? 'none' : 'flex',
                    }}
                    text={t('noMessages')}
                />
            );
        }, [isPlaceHolderDisplay]);

        useEffect(() => {
            if (!isMainConnected || metadata.length <= 0) return noop;
            if (subscribeInfo) {
                const len = subscribeInfo?.length;
                // const obs: CountedSubject<unknown>[] = [];
                for (let i = 0; i < len; i += 1) {
                    const curSubcibe = subscribeInfo[i];
                    const name = curSubcibe?.name;
                    const curItem = metadata.find((item) => item.dataName === name);
                    if (!curItem) {
                        // eslint-disable-next-line no-continue
                        continue;
                    }

                    const connectedSubj: CountedSubject<unknown> = handleSubscribe(curSubcibe);
                    // 2.若包含channel => subcribeToDataWithChannel => obs.push
                    if (connectedSubj) {
                        connectedSubjectsRef.current.push({
                            name,
                            connectedSubject: connectedSubj,
                        });
                    }
                }
                if (!hasSubscribed) {
                    setHasSubscribed(true);
                }

                // eslint-disable-next-line no-restricted-syntax
                for (const subject of connectedSubjectsRef.current) {
                    const subscription = subject.connectedSubject.subscribe((val) => {
                        checkDataEmpty(val);

                        if (initSubscriptionMapRef.current && !isEmpty(initSubscriptionMapRef.current)) {
                            const consumer = initSubscriptionMapRef.current[subject.name].consumer ?? defaultConsumer;
                            consumer(val);
                        }
                    });

                    subscriptionsRef.current.push({
                        name: subject.name,
                        subscription,
                    });
                }
            }

            return () => {
                subscriptionsRef.current.forEach((subscription) => {
                    subscription.subscription.unsubscribe();
                });
            };
        }, [metadata, isMainConnected]);

        useNotifyInitialChanel(panelId, addChannel);
        useUpdateChannel(panelId, addChannel);

        const style = useMemo(
            () => ({
                display: !isPlaceHolderDisplay ? 'block' : 'none',
            }),
            [isPlaceHolderDisplay],
        );

        const contextValue = useMemo(
            () => ({
                panelId,
                initSubscription,
                logger,
                metaData: _panelMetaData,
                splitPanel,
                closePanel,
                updateMetaData,
                enterFullScreen,
                exitFullScreen,
                onPanelResize,
                data: combinedData,
                addChannel,
                updateChannel,
                closeSubcription,
                setKeyDownHandlers,
                setKeyUpHandlers,
                registerFullScreenHooks,
            }),
            [
                logger,
                updateChannel,
                initSubscription,
                _panelMetaData,
                splitPanel,
                closePanel,
                updateMetaData,
                enterFullScreen,
                exitFullScreen,
                onPanelResize,
                combinedData,
                addChannel,
                closeSubcription,
                setKeyDownHandlers,
                setKeyUpHandlers,
                registerFullScreenHooks,
            ],
        );

        return (
            <PanelContext.Provider value={contextValue}>
                <KeyListener active={isSelected} keyDownHandlers={keyDownHandlers} keyUpHandlers={keyUpHandlers} />
                <PanelRoot id={panelId} style={style} onClick={onPanleClick} ref={panelRootRef}>
                    <PanelComponent {...props} />
                </PanelRoot>
                {NoDataPlaceHolder}
            </PanelContext.Provider>
        );
    }

    return Object.assign(React.memo(PanelWrapper), {
        displayName: PanelComponent.displayName || PanelComponent.name,
    });
}
