import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import i18next from '@dreamview/dreamview-lang';
import { PageLayoutStoreProvider } from '@dreamview/dreamview-core/src/store/PageLayoutStore';
import { hmiUtils, usePickHmiStore, CURRENT_MODE, changeMode } from '@dreamview/dreamview-core/src/store/HmiStore';
import { usePanelLayoutStore, updateByMode } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { flushSync } from 'react-dom';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import Menu from '../Menu';
import PanelLayout from '../PanelLayout';
import PlayerControlBar from '../PlayerControlBar';
import MenuDrawer from '../MenuDrawer';
import usePageLayoutStyles from './style';
import PerceptionGuide from '../Guide/PerceptionGuide';
import WelcomeGuide from '../WelcomeGuide';
import DefaultGuide from '../Guide/DefaultGuide';
import useLocalStorage from '../../util/useLocalStorage';

function ChangeLang() {
    return (
        <>
            <div onClick={() => i18next.changeLanguage('zh')}>1、中文</div>
            <div onClick={() => i18next.changeLanguage('en')}>2、英文</div>
        </>
    );
}

function PageLayout() {
    const [hmi, dispatch] = usePickHmiStore();

    const { mainApi, isMainConnected } = useWebSocketServices();

    const [, dispatchLayout] = usePanelLayoutStore();

    const { classes: c, cx, css } = usePageLayoutStyles();

    const [localFirstUseGuideMode, setLocalFirstUseGuideMode] = useLocalStorage<string>('UserFirstUseGuideMode', null);

    const [enterGuideState, setEnterGuideState] = useState({ currentMode: '' });

    const isPlayerControlBarShow = useMemo(() => hmiUtils.isPlayerControlShow(hmi), [hmi]);

    const [perceptionGuideLastStep, setPerceptionGuideLastState] = useState({});

    const controlPerceptionGuideLastStep = useCallback((perceptionGuideLastStepStyle: object) => {
        setPerceptionGuideLastState(perceptionGuideLastStepStyle);
    }, []);

    const setEnterGuideStateMemo = useCallback((currentMode: CURRENT_MODE) => {
        flushSync(() => {
            setEnterGuideState({ currentMode: '' });
        });
        setEnterGuideState(() => ({ currentMode }));
    }, []);

    const clickEnterThisModeToGuide = useCallback(
        (useGuideMode: CURRENT_MODE) => {
            if (isMainConnected) {
                dispatch(
                    changeMode(mainApi, useGuideMode, () => {
                        dispatchLayout(updateByMode({ mode: useGuideMode }));
                    }),
                );
            }

            if (useGuideMode === CURRENT_MODE.DEFAULT) {
                setEnterGuideState({ currentMode: CURRENT_MODE.DEFAULT });
            }
            if (useGuideMode === CURRENT_MODE.PERCEPTION) {
                setEnterGuideState({ currentMode: CURRENT_MODE.PERCEPTION });
            }
        },
        [isMainConnected],
    );

    return (
        <div className={c['dv-root']}>
            {!localFirstUseGuideMode && (
                <WelcomeGuide
                    clickEnterThisModeToGuide={clickEnterThisModeToGuide}
                    setLocalFirstUseGuideMode={setLocalFirstUseGuideMode}
                />
            )}
            {enterGuideState.currentMode === CURRENT_MODE.DEFAULT && <DefaultGuide />}
            {enterGuideState.currentMode === CURRENT_MODE.PERCEPTION && (
                <PerceptionGuide controlPerceptionGuideLastStep={controlPerceptionGuideLastStep} />
            )}
            {localFirstUseGuideMode && (
                <>
                    <div className={c['dv-layout-menu']}>
                        <Menu setEnterGuideStateMemo={setEnterGuideStateMemo} />
                    </div>
                    <div className={c['dv-layout-content']}>
                        <div className={c['dv-layout-window']}>
                            <MenuDrawer />
                            <div className={cx(c['dv-layout-panellayout'])}>
                                <div id='perception-guide-perceived-effects' className={css(perceptionGuideLastStep)} />
                                <PanelLayout />
                            </div>
                        </div>
                        <div
                            className={cx(c['dv-layout-playercontrol'], {
                                [c['dv-layout-playercontrol-active']]: isPlayerControlBarShow,
                            })}
                        >
                            <PlayerControlBar />
                        </div>
                    </div>
                </>
            )}
        </div>
    );
}

function PageLayoutWrapper() {
    return (
        <PageLayoutStoreProvider>
            <PageLayout />
        </PageLayoutStoreProvider>
    );
}
export default React.memo(PageLayoutWrapper);
