import React, { useEffect } from 'react';
import { Subscription } from 'rxjs';
import {
    updateStatus,
    usePickHmiStore,
    updateCurrentMode,
    updateCurrentOperate,
    changeDynamic,
} from '@dreamview/dreamview-core/src/store/HmiStore';
import { ChangeCertStatusAction } from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import { ENUM_CERT_STATUS } from '@dreamview/dreamview-core/src/store/MenuStore';
import { Emptyable, noop } from './util/similarFunctions';
import useWebSocketServices from './services/hooks/useWebSocketServices';
import { StreamDataNames } from './services/api/types';
import { useUserInfoStore } from './store/UserInfoStore';
import { initUserInfo } from './store/UserInfoStore/actions';
import useComponentDisplay from './hooks/useComponentDisplay';
import { menuStoreUtils, useMenuStore } from './store/MenuStore';

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
                })
                .catch(() => {
                    dispatch(ChangeCertStatusAction(ENUM_CERT_STATUS.FAIL));
                });
        }
    }, [isPluginConnected]);

    useEffect(() => {
        if (pluginApi?.getAccountInfo && CertSuccessState) {
            pluginApi?.getAccountInfo().then((res) => {
                dispatchUserInfo(
                    initUserInfo({
                        userInfo: res,
                        isLogin: true,
                    }),
                );
            });
        }
    }, [pluginApi, CertSuccessState]);
}

function useInitHmiStatus() {
    const { isMainConnected, metadata, streamApi } = useWebSocketServices();
    const [, dispatch] = usePickHmiStore();

    useEffect(() => {
        if (!isMainConnected) return noop;

        let subsctiptiion: Emptyable<Subscription>;

        if (metadata.findIndex((item) => item.dataName === StreamDataNames.HMI_STATUS) > -1) {
            subsctiptiion = streamApi?.subscribeToData(StreamDataNames.HMI_STATUS).subscribe((data) => {
                dispatch(updateStatus(data as any));
            });
        }
        return () => {
            subsctiptiion?.unsubscribe();
        };
    }, [metadata]);
}

function useInitAppData() {
    const { isMainConnected, mainApi } = useWebSocketServices();
    const [, dispatch] = usePickHmiStore();

    useEffect(() => {
        if (isMainConnected) {
            mainApi.loadRecords();
            mainApi.loadScenarios();
            mainApi.loadRTKRecords();
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

export default function InitAppData() {
    useInitHmiStatus();
    useInitUserMixInfo();
    useInitAppData();
    useInitDynamic();
    return <></>;
}
