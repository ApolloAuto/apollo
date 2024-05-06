/* eslint-disable @typescript-eslint/no-empty-function */
/* eslint-disable react/jsx-no-useless-fragment */
import React, {
    PropsWithChildren,
    useCallback,
    useContext,
    useEffect,
    useLayoutEffect,
    useMemo,
    useRef,
    useState,
} from 'react';
import { Spin, useImagePrak } from '@dreamview/dreamview-ui';
import { usePanelCatalogContext } from '@dreamview/dreamview-core/src/store/PanelCatalogStore';
import ErrorBoundary from '@dreamview/dreamview-core/src/components/ErrorBoundery';
import { MosaicPath, MosaicWindow, MosaicWindowContext } from 'react-mosaic-component';
import { CustomizeEvent, useEventHandlersContext } from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import useRegisterUpdateChanel from '@dreamview/dreamview-core/src/hooks/useRegisterUpdateChanel';
import { FullScreenFnRef } from '@dreamview/dreamview-core/src/store/PanelInnerStore/type';
import { PanelTileProvider } from '@dreamview/dreamview-core/src/store/PanelInnerStore/PanelTileStore';
import { hooksManager } from '@dreamview/dreamview-core/src/util/HooksManager';
import { useTranslation } from 'react-i18next';
import classNames from 'classnames';
import { refreshPanel, usePanelLayoutStore } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { usePanelInfoStore } from '@dreamview/dreamview-core/src/store/PanelInfoStore';
import useStyle from './style';
import RenderToolbar from '../RenderToolbar';
import { FullScreen } from '../FullScreen';

import './index.less';
import { usePickHmiStore } from '../../../../store/HmiStore';

export interface RenderTileProps {
    // panelId，panel实例化唯一Id
    panelId: string;
    path: MosaicPath;
}

interface ChildProps {
    path: MosaicPath;
    panelId: string;
}

function Child(props: PropsWithChildren<ChildProps>) {
    const { path, panelId } = props;
    const { t } = useTranslation('panels');
    const { classes, cx } = useStyle();
    const elem = useRef<HTMLDivElement>();
    const [dragging, setDraging] = useState(false);
    const [, dispatch] = usePanelLayoutStore();
    const [hmi] = usePickHmiStore();
    const [panelInfoState, panelInfoDispatch] = usePanelInfoStore();
    const isSelected = useMemo(() => panelInfoState?.selectedPanelIds?.has(panelId) ?? false, [panelInfoState]);
    const {
        // connectDragPreview是手动修改node_modules下的react-mosaic-component的代码后新增的api
        // 通过patch-packages工具可记录下node_modules下的文件变更的内容，并生成patch文件，patch文件在fronted/patches下
        // 所以重装依赖或第一次安装依赖后需要运行yarn patch-package命令，运行命令后patch文件记录的变更内容将同步到node_modules下的react-mosaic-component的代码中
        // @ts-ignore
        mosaicWindowActions: { connectDragPreview },
    } = useContext(MosaicWindowContext);

    const [panelElem, toolBarElem] = props.children as React.ReactNode[];

    const onRef = (el: HTMLDivElement) => {
        connectDragPreview(el as any);
        elem.current = el;
    };

    function bindDragHandler() {
        function dragStartHandler() {
            setDraging(true);
        }
        function dragEndHandler() {
            setDraging(false);
        }
        elem.current.addEventListener('dragstart', dragStartHandler);
        elem.current.addEventListener('dragend', dragEndHandler);
    }

    useEffect(() => {
        bindDragHandler();
    }, []);

    const onRefresh = () => {
        dispatch(refreshPanel({ mode: hmi.currentMode, path }));
    };

    const errorImg = useImagePrak('ic_fail_to_load');
    const errorComponent = (
        <div className={classes['panel-error']}>
            <div>
                <img alt='' src={errorImg} />
                <p>
                    {t('panelErrorMsg1')}
                    &nbsp;
                    <span onClick={onRefresh}>{t('panelErrorMsg2')}</span>
                    &nbsp;
                    {t('panelErrorMsg3')}
                </p>
            </div>
        </div>
    );

    const finalCls = `${cx(classes['panel-item-container'], {
        [classes.dragging]: dragging,
    })} ${classNames('panel-container', {
        'panel-selected': isSelected,
    })}`;

    return (
        <div className={finalCls} ref={onRef}>
            {toolBarElem}
            <div className={classes['panel-item-inner']}>
                <ErrorBoundary errComponent={errorComponent}>{panelElem}</ErrorBoundary>
            </div>
        </div>
    );
}

const ChildMemo = React.memo(Child);

function RenderTile(props: RenderTileProps) {
    const { panelId, path } = props;
    const [inFullScreen, _setFullScreen] = useState<boolean>(false);
    const eventHandlers = useEventHandlersContext();
    const fullScreenEventRef = useRef<CustomizeEvent>();
    const updateChannel = useRegisterUpdateChanel(panelId);
    const fullScreenFnRef = useRef<FullScreenFnRef>({
        enterFullScreen: async () => {},
        exitFullScreen: async () => {},
    });

    const { panelCatalog, panelComponents } = usePanelCatalogContext();

    const PanelComponent = panelComponents.get(panelId);

    const panel = panelCatalog.get(panelId);

    const CustomRenderToolBar = panel?.renderToolbar;

    const toolBar = useMemo(() => {
        if (CustomRenderToolBar) {
            return (
                <CustomRenderToolBar
                    path={path}
                    panel={panel}
                    panelId={panelId}
                    inFullScreen={inFullScreen}
                    updateChannel={updateChannel}
                />
            );
        }

        return (
            <RenderToolbar
                path={path}
                panel={panel}
                panelId={panelId}
                inFullScreen={inFullScreen}
                updateChannel={updateChannel}
            />
        );
    }, [CustomRenderToolBar, path, panel, panelId, inFullScreen, updateChannel]);

    useLayoutEffect(() => {
        const { customizeSubs } = eventHandlers;
        customizeSubs.reigisterCustomizeEvent(`full:screen:${panelId}`);
        fullScreenEventRef.current = customizeSubs.getCustomizeEvent(`full:screen:${panelId}`);
        fullScreenEventRef.current.subscribe((isFullScreen) => {
            _setFullScreen(isFullScreen as boolean);
        });
    }, [eventHandlers, panelId]);

    const enterFullScreen = useCallback(async () => {
        if (inFullScreen) return;

        const fullScreenHookConfig = hooksManager.getHook(panelId);
        let hookBeforeResBoolean = false;

        if (fullScreenHookConfig?.beforeEnterFullScreen) {
            hookBeforeResBoolean = await hooksManager.handleFullScreenBeforeHook(
                fullScreenHookConfig.beforeEnterFullScreen,
            );
        }

        const isExecutable = !fullScreenHookConfig?.beforeEnterFullScreen || hookBeforeResBoolean;

        if (isExecutable) {
            _setFullScreen(true);

            if (fullScreenHookConfig?.afterEnterFullScreen) {
                fullScreenHookConfig?.afterEnterFullScreen();
            }
        }
    }, [inFullScreen, panelId]);

    const exitFullScreen = useCallback(async () => {
        if (!inFullScreen) return;

        const fullScreenHookConfig = hooksManager.getHook(panelId);
        let hookBeforeResBoolean = false;

        if (fullScreenHookConfig?.beforeExitFullScreen) {
            hookBeforeResBoolean = await hooksManager.handleFullScreenBeforeHook(
                fullScreenHookConfig.beforeEnterFullScreen,
            );
        }

        const isExecutable = !fullScreenHookConfig?.beforeEnterFullScreen || hookBeforeResBoolean;

        if (isExecutable) {
            _setFullScreen(false);

            if (fullScreenHookConfig?.afterExitFullScreen) {
                fullScreenHookConfig?.afterExitFullScreen();
            }
        }
    }, [inFullScreen]);

    useEffect(() => {
        fullScreenFnRef.current.enterFullScreen = enterFullScreen;
        fullScreenFnRef.current.exitFullScreen = exitFullScreen;
    }, [enterFullScreen, exitFullScreen]);

    if (!PanelComponent) {
        return (
            <MosaicWindow<string> path={path} title='unknown'>
                <PanelTileProvider
                    path={path}
                    enterFullScreen={enterFullScreen}
                    exitFullScreen={exitFullScreen}
                    fullScreenFnObj={fullScreenFnRef.current}
                >
                    <RenderToolbar
                        path={path}
                        panel={panel}
                        panelId={panelId}
                        inFullScreen={inFullScreen}
                        updateChannel={updateChannel}
                    />
                    unknown component type：
                    {panelId}
                </PanelTileProvider>
            </MosaicWindow>
        );
    }

    return (
        <FullScreen enabled={inFullScreen}>
            <MosaicWindow path={path} title={panel?.title}>
                <ErrorBoundary>
                    <React.Suspense fallback={<Spin />}>
                        <PanelTileProvider
                            path={path}
                            enterFullScreen={enterFullScreen}
                            exitFullScreen={exitFullScreen}
                            fullScreenFnObj={fullScreenFnRef.current}
                        >
                            <ChildMemo path={path} panelId={panelId}>
                                <PanelComponent panelId={panelId} />
                                {toolBar}
                            </ChildMemo>
                        </PanelTileProvider>
                    </React.Suspense>
                </ErrorBoundary>
            </MosaicWindow>
        </FullScreen>
    );
}

export default React.memo(RenderTile);
