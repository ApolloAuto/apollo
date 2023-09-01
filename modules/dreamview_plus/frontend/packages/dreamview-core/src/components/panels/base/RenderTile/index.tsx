/* eslint-disable react/jsx-no-useless-fragment */
import React, { PropsWithChildren, useContext, useEffect, useLayoutEffect, useMemo, useRef, useState } from 'react';
import { Spin } from '@dreamview/dreamview-ui';
import { usePanelCatalogContext } from '@dreamview/dreamview-core/src/store/PanelCatalogStore';
import { IPanelMetaInfo } from '@dreamview/dreamview-core/src/components/panels/type/Panel';
import ErrorBoundary from '@dreamview/dreamview-core/src/components/ErrorBoundery';
import { MosaicPath, MosaicWindow, MosaicWindowContext } from 'react-mosaic-component';
import { PanelInnerProvider } from '@dreamview/dreamview-core/src/store/PanelInnerStore';
import { CustomizeEvent, useEventHandlersContext } from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import useRegisterUpdateChanel from '@dreamview/dreamview-core/src/hooks/useRegisterUpdateChanel';
import { createLocalStoragePersistor } from '@dreamview/dreamview-core/src/store/base/Persistor';
import { PanelInnerEventEmitterProvider } from '@dreamview/dreamview-core/src/store/PanelInnerStore/eventEmitter';
import errorImg from '@dreamview/dreamview-core/src/assets/ic_fail_to_load.png';
import { useTranslation } from 'react-i18next';
import classNames from 'classnames';
import { refreshPanel, usePanelLayoutStore } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import useStyle from './style';
import RenderToolbar from '../RenderToolbar';
import { FullScreen } from '../FullScreen';
import { SubscribeInfo } from '../../type/RenderToolBar';
import { usePanelInfoStore } from '../../../../store/PanelInfoStore';
import './index.less';

export interface RenderTileProps {
    // panelId，panel实例化唯一Id
    panelId: string;
    path: MosaicPath;
}

interface ChildProps {
    path: MosaicPath;
    panel: IPanelMetaInfo;
    panelId: string;
    inFullScreen: boolean;
    updateChannel: (newChannel: SubscribeInfo) => void;
}

function Child(props: PropsWithChildren<ChildProps>) {
    const { path, panelId } = props;
    const { t } = useTranslation('panels');
    const { classes, cx } = useStyle();
    const elem = useRef<HTMLDivElement>();
    const [dragging, setDraging] = useState(false);
    const [, dispatch] = usePanelLayoutStore();
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
        dispatch(refreshPanel({ path }));
    };

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

export default function RenderTile(props: RenderTileProps) {
    const { panelId, path } = props;
    const [inFullScreen, _setFullScreen] = useState<boolean>(false);
    const eventHandlers = useEventHandlersContext();
    const fullScreenEventRef = useRef<CustomizeEvent>();
    const updateChannel = useRegisterUpdateChanel(panelId);

    const { panelCatalog, panelComponents, panelToolBar } = usePanelCatalogContext();

    const PanelComponent = panelComponents.get(panelId);

    const panel = panelCatalog.get(panelId);

    // const CustomRenderToolBar = panelToolBar.get(panelId);
    const CustomRenderToolBar = panel.renderToolbar;

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

    const [{ initialState, persistor }] = useState(() => ({
        persistor: createLocalStoragePersistor(`dv-panel-inner-stoer-${panelId}`),
        initialState: {
            panelId,
            state: {},
        },
    }));

    if (!PanelComponent) {
        return (
            <MosaicWindow<string> path={path} title='unknown'>
                <RenderToolbar
                    path={path}
                    panel={panel}
                    panelId={panelId}
                    inFullScreen={inFullScreen}
                    updateChannel={updateChannel}
                />
                unknown component type：
                {panelId}
            </MosaicWindow>
        );
    }

    return (
        <FullScreen enabled={inFullScreen}>
            <MosaicWindow path={path} title={panel.title}>
                <ErrorBoundary>
                    <React.Suspense fallback={<Spin />}>
                        <PanelInnerEventEmitterProvider>
                            <PanelInnerProvider persistor={persistor} initialState={initialState}>
                                <Child
                                    path={path}
                                    panel={panel}
                                    panelId={panelId}
                                    inFullScreen={inFullScreen}
                                    updateChannel={updateChannel}
                                >
                                    <PanelComponent panelId={panelId} />
                                    {toolBar}
                                </Child>
                            </PanelInnerProvider>
                        </PanelInnerEventEmitterProvider>
                    </React.Suspense>
                </ErrorBoundary>
            </MosaicWindow>
        </FullScreen>
    );
}
