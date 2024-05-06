import React, { useCallback, useState } from 'react';
import { PageLayoutStoreProvider } from '@dreamview/dreamview-core/src/store/PageLayoutStore';
import { usePickHmiStore, CURRENT_MODE } from '@dreamview/dreamview-core/src/store/HmiStore';
import { flushSync } from 'react-dom';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { useLocalStorageState, KEY_MANAGER } from '@dreamview/dreamview-core/src/util/storageManager';
import useComponentDisplay from '@dreamview/dreamview-core/src/hooks/useComponentDisplay';
import { useMenuStore } from '@dreamview/dreamview-core/src/store/MenuStore';
import { useAppInitContext } from '@dreamview/dreamview-core/src/store/AppInitStore';
import { ENUM_MENU_KEY } from '@dreamview/dreamview-core/src/store/MenuStore/actionTypes';
import Menu from '../Menu';
import PanelLayout from '../PanelLayout';
import MenuDrawer from '../MenuDrawer';
import usePageLayoutStyles from './style';
import PerceptionGuide from '../Guide/PerceptionGuide';
import WelcomeGuide from '../WelcomeGuide';
import DefaultGuide from '../Guide/DefaultGuide';
import PNCGuide from '../Guide/PNCGuide';
import VehicleGuide from '../Guide/VehicleGuide';
import BottomBar from '../BottomBar';
import PageLoading from './PageLoading';
import { IfComponents } from '../panels/Charts/Div';

const IfPageLoading = IfComponents(PageLoading);

function PageLayout() {
    const { hasInit } = useAppInitContext();

    const [hmi] = usePickHmiStore();

    const [{ activeMenu }] = useMenuStore();

    const hidepanel = activeMenu === ENUM_MENU_KEY.PROFILE_MANAGEER;

    const { isMainConnected } = useWebSocketServices();

    const { classes: c, cx, css } = usePageLayoutStyles();

    const [localFirstUseGuideMode, setLocalFirstUseGuideMode] = useLocalStorageState<string>(
        KEY_MANAGER.UserFirstUseGuideMode,
    );

    const [enterGuideState, setEnterGuideState] = useState({ currentMode: '' });

    const [{ isBottomBarShow }] = useComponentDisplay();

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
            if (useGuideMode === CURRENT_MODE.DEFAULT) {
                setEnterGuideState({ currentMode: CURRENT_MODE.DEFAULT });
            }
            if (useGuideMode === CURRENT_MODE.PERCEPTION) {
                setEnterGuideState({ currentMode: CURRENT_MODE.PERCEPTION });
            }
            if (useGuideMode === CURRENT_MODE.PNC) {
                setEnterGuideState({ currentMode: CURRENT_MODE.PNC });
            }
            if (useGuideMode === CURRENT_MODE.VEHICLE_TEST) {
                setEnterGuideState({ currentMode: CURRENT_MODE.VEHICLE_TEST });
            }
        },
        [isMainConnected],
    );
    return (
        <div className={c['dv-root']}>
            <IfPageLoading rif={!hasInit} />
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
            {enterGuideState.currentMode === CURRENT_MODE.PNC && <PNCGuide />}
            {enterGuideState.currentMode === CURRENT_MODE.VEHICLE_TEST && (
                <VehicleGuide currentOperation={hmi.currentOperation} />
            )}
            {localFirstUseGuideMode && (
                <>
                    <div className={c['dv-layout-menu']}>
                        <Menu setEnterGuideStateMemo={setEnterGuideStateMemo} />
                    </div>
                    <div className={c['dv-layout-content']}>
                        <div className={c['dv-layout-window']}>
                            <MenuDrawer />
                            <div
                                style={{ display: hidepanel ? 'none' : 'block' }}
                                className={cx(c['dv-layout-panellayout'])}
                            >
                                <div id='perception-guide-perceived-effects' className={css(perceptionGuideLastStep)} />
                                <PanelLayout />
                            </div>
                        </div>
                        <div
                            className={cx(c['dv-layout-playercontrol'], {
                                [c['dv-layout-playercontrol-active']]: isBottomBarShow,
                            })}
                        >
                            <BottomBar />
                        </div>
                    </div>
                </>
            )}
        </div>
    );
}

const PageLayoutMemo = React.memo(PageLayout);

function PageLayoutWrapper() {
    return (
        <PageLayoutStoreProvider>
            <PageLayoutMemo />
        </PageLayoutStoreProvider>
    );
}
export default React.memo(PageLayoutWrapper);
