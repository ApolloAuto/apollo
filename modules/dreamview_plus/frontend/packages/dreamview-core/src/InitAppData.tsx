import React, { useEffect, useRef } from 'react';
import {
    updateStatus,
    usePickHmiStore,
    updateCurrentMode,
    updateCurrentOperate,
    changeDynamic,
} from '@dreamview/dreamview-core/src/store/HmiStore';
import { ChangeCertStatusAction } from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import { ENUM_CERT_STATUS } from '@dreamview/dreamview-core/src/store/MenuStore';
import {
    webSocketManager,
    IEventName,
    ConnectionStatusEnum,
} from '@dreamview/dreamview-core/src/services/WebSocketManager';
import DreamviewConfig from '@dreamview/dreamview-core/package.json';
import { useRegistryInitEvent, IAppInitStatus } from '@dreamview/dreamview-core/src/store/AppInitStore';
import { message } from '@dreamview/dreamview-ui';
import { noop } from './util/similarFunctions';
import useWebSocketServices from './services/hooks/useWebSocketServices';
import { StreamDataNames } from './services/api/types';
import { useUserInfoStore } from './store/UserInfoStore';
import { updateSubscribe, initUserInfo } from './store/UserInfoStore/actions';
import useComponentDisplay from './hooks/useComponentDisplay';
import { menuStoreUtils, useMenuStore } from './store/MenuStore';

// @ts-ignore
window.dreamviewVersion = DreamviewConfig.version;

function useInitProto() {
    const changeHandler = useRegistryInitEvent('Proto Parsing', 2);
    useEffect(() => {
        let channelTotal = -1;
        const baseProtoTotal = webSocketManager.initProtoFiles.length;
        let channelLeaveCount = -1;
        let baseProtoLeaveCount = webSocketManager.initProtoFiles.length;
        changeHandler({
            status: IAppInitStatus.LOADING,
            progress: 0,
        });
        function processchange() {
            if (channelTotal === -1 || baseProtoTotal === -1) {
                return;
            }
            if (channelLeaveCount === 0 && baseProtoLeaveCount === 0) {
                changeHandler({
                    status: IAppInitStatus.DONE,
                    progress: 100,
                });
            } else {
                changeHandler({
                    status: IAppInitStatus.LOADING,
                    progress: Math.floor(
                        ((channelTotal + baseProtoTotal - channelLeaveCount - baseProtoLeaveCount) /
                            (channelTotal + baseProtoTotal)) *
                            100,
                    ),
                });
            }
        }
        function ChannelTotalHandler(num: number) {
            channelTotal = num;
            channelLeaveCount = num;
        }
        function ChannelChangeHandler() {
            channelLeaveCount -= 1;
            if (channelLeaveCount === 0) {
                webSocketManager.removeEventListener(IEventName.ChannelChange, ChannelChangeHandler);
            }
            processchange();
        }
        function BaseProtoChangeHandler() {
            baseProtoLeaveCount -= 1;
            if (baseProtoLeaveCount === 0) {
                webSocketManager.removeEventListener(IEventName.ChannelChange, BaseProtoChangeHandler);
            }
            processchange();
        }

        webSocketManager.addEventListener(IEventName.ChannelTotal, ChannelTotalHandler);

        webSocketManager.addEventListener(IEventName.ChannelChange, ChannelChangeHandler);

        webSocketManager.addEventListener(IEventName.BaseProtoChange, BaseProtoChangeHandler);
    }, []);
}
function useInitWebSocket() {
    const changeHandler = useRegistryInitEvent('Websocket Connect', 1);
    useEffect(() => {
        let progress = 0;
        // eslint-disable-next-line @typescript-eslint/no-shadow
        let message = 'Websocket Connecting...';
        let websocketStatus = IAppInitStatus.LOADING;
        const timer = setInterval(() => {
            progress += 2;
            if (progress >= 100) {
                if (websocketStatus !== IAppInitStatus.DONE) {
                    websocketStatus = IAppInitStatus.FAIL;
                    message = 'Websocket Connect Failed';
                    progress = 99;
                } else {
                    progress = 100;
                }
            }
            if (websocketStatus === IAppInitStatus.FAIL) {
                clearInterval(timer);
            }
            changeHandler({
                status: websocketStatus,
                progress,
                message,
            });
        }, 100);

        webSocketManager.mainConnection.connectionStatus$.subscribe((status) => {
            if (status === ConnectionStatusEnum.CONNECTED) {
                websocketStatus = IAppInitStatus.LOADING;
                progress = Math.max(progress, 66);
                message = 'Receiving Metadata...';
            }
            if (status === ConnectionStatusEnum.CONNECTING) {
                websocketStatus = IAppInitStatus.LOADING;
                message = 'Websocket Connecting...';
            }
            if (status === ConnectionStatusEnum.DISCONNECTED) {
                websocketStatus = IAppInitStatus.FAIL;
                message = 'Websocket Connect Failed';
            }
            if (status === ConnectionStatusEnum.METADATA) {
                progress = 100;
                message = 'Metadata Receive Successful!';
                websocketStatus = IAppInitStatus.DONE;
            }
        });
        return () => {
            clearInterval(timer);
        };
    }, []);
}
function useInitUserMixInfo() {
    const [, dispatchUserInfo] = useUserInfoStore();
    const [{ certStatus }, dispatch] = useMenuStore();
    const { isPluginConnected, pluginApi } = useWebSocketServices();
    const CertSuccessState = menuStoreUtils.isCertSuccess(certStatus);

    useEffect(() => {
        if (isPluginConnected) {
            pluginApi
                .checkCertStatus()
                .then(() => {
                    dispatch(ChangeCertStatusAction(ENUM_CERT_STATUS.SUCCESS));
                    dispatchUserInfo(
                        initUserInfo({
                            userInfo: {
                                avatar_url: undefined,
                                displayname: undefined,
                                id: undefined,
                            },
                            isLogin: true,
                        }),
                    );
                    pluginApi.getSubscribeList({
                        useCache: true,
                    });
                })
                .catch(() => {
                    dispatch(ChangeCertStatusAction(ENUM_CERT_STATUS.FAIL));
                });
            dispatchUserInfo(updateSubscribe(pluginApi));
        }
    }, [isPluginConnected]);

    const event = useRef<any>({});
    const eventValue = useRef<any>({});
    const addDreamviewEventListener = (name: string, callback: any) => {
        if (!event.current[name]) {
            event.current[name] = [];
        }
        event.current[name].push(callback);
        if (eventValue.current[name]) {
            try {
                callback(eventValue.current[name]);
            } catch (err) {
                console.error(err);
            }
        }
    };

    const removeDreamviewEventListener = (name: string, callback: any) => {
        if (!event.current[name]) {
            event.current[name] = [];
        }
        event.current[name] = event.current[name].filter((item: any) => callback !== item);
    };

    const dispatchDreamviewEvent = (name: string, value: any, memo?: any) => {
        if (memo) {
            eventValue.current[name] = value;
        }
        if (event.current[name]) {
            event.current[name].forEach((cb: any) => {
                try {
                    cb(value);
                } catch (err) {
                    console.error(err);
                }
            });
        }
    };

    useEffect(() => {
        // @ts-ignore
        window.addDreamviewEventListener = addDreamviewEventListener;
        // @ts-ignore
        window.removeDreamviewEventListener = removeDreamviewEventListener;
    }, []);

    useEffect(() => {
        if (pluginApi?.getAccountInfo && CertSuccessState) {
            pluginApi
                ?.getAccountInfo()
                .then((res) => {
                    dispatchUserInfo(
                        initUserInfo({
                            userInfo: res,
                            isLogin: true,
                        }),
                    );
                    dispatchDreamviewEvent('dreamviewUserInfo', res, true);
                })
                .catch((e: any) => {
                    // console.log(e.data.info.message);
                    message({ type: 'error', content: e?.data?.info?.message || '登陆异常', duration: 3 });
                });
        }
    }, [pluginApi, CertSuccessState]);
}

function useInitHmiStatus() {
    const { isMainConnected, metadata, streamApi } = useWebSocketServices();
    const [, dispatch] = usePickHmiStore();

    useEffect(() => {
        if (!isMainConnected) return noop;

        const subscription = streamApi?.subscribeToData(StreamDataNames.HMI_STATUS).subscribe((data) => {
            dispatch(
                updateStatus({
                    ...(data as any),
                    frontendIsFromHmi: true,
                } as any),
            );
        });
        return () => {
            subscription?.unsubscribe();
        };
    }, [metadata]);
}

function useInitAppData() {
    const { isMainConnected, mainApi } = useWebSocketServices();
    const [, dispatch] = usePickHmiStore();

    useEffect(() => {
        if (isMainConnected) {
            mainApi.loadRecords();
            // fixme： 需要针对仿真调整
            // mainApi.loadScenarios();
            mainApi.loadRTKRecords();
            mainApi.loadMaps();
            mainApi.getInitData().then((r) => {
                dispatch(updateCurrentMode(r.currentMode));
                dispatch(updateCurrentOperate(r.currentOperation));
            });
        }
    }, [isMainConnected]);
}

function useInitDynamic() {
    const [, dispatch] = usePickHmiStore();
    const { mainApi } = useWebSocketServices();
    const [{ isDynamicalModelsShow }] = useComponentDisplay();
    useEffect(() => {
        if (isDynamicalModelsShow) {
            mainApi?.loadDynamic();
            // 这里需要默认选中Simulation Perfect Control
            dispatch(changeDynamic(mainApi, 'Simulation Perfect Control'));
        }
    }, [isDynamicalModelsShow, mainApi]);
}

function useInitDreamviewVersion() {
    useEffect(() => {
        // 创建一个隐藏的div存放版本号
        const versionDiv = document.createElement('div');
        versionDiv.style.display = 'none';
        versionDiv.id = 'dreamviewVersion';
        versionDiv.innerHTML = DreamviewConfig.version;
        document.body.appendChild(versionDiv);
    }, []);
}

export default function InitAppData() {
    useInitHmiStatus();
    useInitUserMixInfo();
    useInitAppData();
    useInitDynamic();
    useInitProto();
    useInitWebSocket();
    useInitDreamviewVersion();
    return <></>;
}
