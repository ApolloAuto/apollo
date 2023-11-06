import React, { PropsWithChildren, useCallback, useEffect, useMemo, useRef } from 'react';
import { Select, SwitchLoading, Tabs, IconIcSucceed, IconIcLoading, message } from '@dreamview/dreamview-ui';
import {
    usePickHmiStore,
    hmiUtils,
    THROTTLE_TIME,
    IScenarioInfo,
    changeMode,
    changeOperate,
    changeRecorder,
    changeScenarios,
    changeMap,
    changeVehicle,
    IInitState,
    HMIModeOperation,
    CURRENT_MODE,
} from '@dreamview/dreamview-core/src/store/HmiStore';
import SourceEmptyImg from '@dreamview/dreamview-core/src/assets/ic_default_page_no_data.png';
import throttle from 'lodash/throttle';
import Logger from '@dreamview/log';
import { useTranslation } from 'react-i18next';
import useComponentDisplay from '@dreamview/dreamview-core/src/hooks/useComponentDisplay';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { useMainApi } from '@dreamview/dreamview-core/src/store/WebSocketManagerStore';
import { usePanelLayoutStore, updateByMode } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import {
    ChangeAdsResourcesAction,
    ChangeEnviormentResourcesAction,
} from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import {
    ENUM_ADS_MANAGER_TAB_KEY,
    ENUM_ENVIORMENT_MANAGER_TAB_KEY,
    useMenuStore,
} from '@dreamview/dreamview-core/src/store/MenuStore';
import MenuDrawerTitle from '../Common/MenuDrawerTitle';
import ModeSettingTitle from './Title';
import useStyle, { useCurrentResourceStyle, useGuideContainerStyle } from './useStyle';
import Divider from './Divider';
import Button from './Button';
import { SourceList1, SourceList2 } from './SourceList';
import CustomScroll from '../../CustomScroll';

const logger = Logger.getInstance('ModeSetting');

function Mode() {
    const { isMainConnected, mainApi } = useWebSocketServices();

    const [hmi, dispatch] = usePickHmiStore();
    const [, dispatchLayout] = usePanelLayoutStore();
    const { classes } = useStyle()();

    const { t } = useTranslation('modeSettings');

    const changeLayout = (mode: string) => {
        dispatchLayout(updateByMode({ mode }));
    };

    const onChange = useCallback(
        (v: string) => {
            if (isMainConnected) {
                dispatch(changeMode(mainApi, v, changeLayout));
            }
        },
        [isMainConnected],
    );
    const nameMap: Record<string, string> = {
        [CURRENT_MODE.PNC]: 'PnC',
    };
    const options = useMemo(
        () => hmi.modes?.map((item) => ({ label: nameMap[item] || item, value: item })) || [],
        [hmi.modes],
    );

    return (
        <>
            <ModeSettingTitle title={t('mode')} />
            <Select onChange={onChange} value={hmi.currentMode} className={classes.space20px} options={options} />
            <Divider />
        </>
    );
}

const ModeMemo = React.memo(Mode);

function Operations() {
    const { isMainConnected, mainApi } = useWebSocketServices();
    const [hmi, dispatch] = usePickHmiStore();
    const { classes } = useStyle()();

    const { t } = useTranslation('modeSettings');

    const options = useMemo(
        () => hmi.operations?.map((item) => ({ label: item, value: `${item}` })) || [],
        [hmi.operations],
    );

    const onChange = useCallback(
        (operate: string) => {
            dispatch(changeOperate(mainApi, operate));
        },
        [isMainConnected],
    );

    return (
        <>
            <ModeSettingTitle title={t('operations')} />
            <Select onChange={onChange} value={hmi.currentOperation} className={classes.space20px} options={options} />
            <Divider />
        </>
    );
}

const OperationsMemo = React.memo(Operations);

function CustomSwitch(props: { moduleName: string }) {
    const { moduleName } = props;

    const { t } = useTranslation('modeSettings');
    const mainApi = useMainApi();
    const [hmi] = usePickHmiStore();

    const moduleStatus = hmi.moduleStatus.get(moduleName);
    const isModuleLocked = hmi.modulesLock.get(moduleName);

    const prevModuleStatus = useRef(moduleStatus);
    const prevIsModuleLocked = useRef(isModuleLocked);

    useEffect(() => {
        logger.debug('hmiStatus', hmi);
        // 状态机： isModuleLocked 从1 变为 0， moduleStatus 从 0 变为 0 ,提示用户monitor没开启
        if (prevIsModuleLocked.current && !isModuleLocked) {
            if (!prevModuleStatus.current && !moduleStatus) {
                message({
                    type: 'warning',
                    content: `${moduleName} ${t('moduleStartupFailed')}`,
                });
            }
        }

        // 状态机：sModuleLocked 从 0 变为 0, moduleStatus 从 1 变为 0 自动关闭，模块挂了
        if (!prevIsModuleLocked.current && !isModuleLocked) {
            if (prevModuleStatus.current && !moduleStatus) {
                if (!hmi.backendShutdown) {
                    message({
                        type: 'warning',
                        content: `${moduleName} ${t('moduleStartupFailed')}`,
                    });
                }
            }
        }

        prevModuleStatus.current = moduleStatus;
        prevIsModuleLocked.current = isModuleLocked;
    }, [moduleStatus, isModuleLocked, moduleName, hmi.backendShutdown]);

    const onChange = useCallback(
        throttle(() => {
            if (moduleStatus) {
                mainApi?.stopModule(moduleName);
            } else {
                mainApi?.startModule(moduleName);
            }
        }, THROTTLE_TIME),
        [mainApi, moduleName, moduleStatus],
    );

    const getLoadingIcon = useCallback(() => {
        if (isModuleLocked) {
            return <IconIcLoading spin className='switch-inner-loading-icon' />;
        }
        return null;
    }, [mainApi, moduleName, isModuleLocked]);

    const loadingChild = useMemo(getLoadingIcon, [getLoadingIcon]);

    return (
        <SwitchLoading
            onChange={onChange}
            checkedChildren={loadingChild}
            unCheckedChildren={loadingChild}
            checked={!!hmi.moduleStatus.get(moduleName)}
            disabled={
                hmiUtils.isCalibrationMode(hmi) && moduleName === hmiUtils.preConditionModule(hmi)
                    ? !hmiUtils.allMonitoredComponentSuccess(hmi)
                    : false
            }
        />
    );
}

const CustomSwitchMemo = React.memo(CustomSwitch);

function Modules() {
    const { classes } = useStyle()();

    const { isMainConnected, mainApi } = useWebSocketServices();
    const [hmi] = usePickHmiStore();

    const { t } = useTranslation('modeSettings');

    const modules = Array.from(hmi.moduleStatus.keys());

    const isLocked = useMemo(() => modules.some((key) => hmi.modulesLock.get(key)), [hmi.modulesLock, modules]);

    // fixme 1期需求补充开关逻辑
    const onSetupAll = useCallback(() => {
        if (isLocked) return;
        // modeSetting -> modules模块
        // setup all
        if (!isMainConnected) return;
        mainApi?.setupAllModule();
    }, [isLocked, isMainConnected, mainApi]);

    const onResetAll = useCallback(() => {
        if (isLocked) return;
        // modeSetting -> modules模块
        // reset all
        if (!isMainConnected) return;
        mainApi?.resetAllModule();
    }, [isLocked, isMainConnected, mainApi]);

    const expendChild = (
        <>
            <div className={classes['modules-operation']}>
                <Button onClick={onSetupAll} width={153}>
                    {t('setupAll')}
                </Button>
                <Button onClick={onResetAll} width={153}>
                    {t('resetAll')}
                </Button>
            </div>
            <div className={classes['modules-switch-container']}>
                {modules.map((key) => (
                    <div className={classes['modules-switch-item']} key={key}>
                        <label>
                            <CustomSwitchMemo moduleName={key} />
                            <span title={key} className={classes['modules-switch-text']}>
                                {key}
                            </span>
                        </label>
                    </div>
                ))}
            </div>
        </>
    );

    return (
        <>
            <ModeSettingTitle expendChild={expendChild} title={t('modules')} />

            <Divider />
        </>
    );
}

const ModulesMemo = React.memo(Modules);

function Recorders() {
    const { isMainConnected, mainApi } = useWebSocketServices();
    const [hmi, dispatch] = usePickHmiStore();
    const items = useMemo(
        () =>
            Object.keys(hmi.records)
                .map((item) => ({
                    id: item,
                    label: item,
                    content: item,
                }))
                .sort((a, b) => a.id.localeCompare(b.id)),
        [hmi.records],
    );
    const onChange = useCallback(
        (recorderId: string) => {
            if (isMainConnected) {
                dispatch(changeRecorder(mainApi, recorderId));
            }
        },
        [isMainConnected],
    );

    return <SourceList1<string> activeId={hmi?.currentRecordId} onChange={onChange} items={items} />;
}

const RecordersMemo = React.memo(Recorders);

function Scenarios() {
    const [hmi, dispatch] = usePickHmiStore();
    const { isMainConnected, mainApi } = useWebSocketServices();
    const onChange = useCallback(
        (scenariosSet: any, scenarios: any) => {
            if (isMainConnected) {
                dispatch(
                    changeScenarios(mainApi, {
                        scenariosSetId: scenariosSet.id,
                        scenarioId: scenarios.scenarioId,
                    }),
                );
            }
        },
        [isMainConnected],
    );

    const items = useMemo(
        () =>
            Object.keys(hmi.scenarioSet || {})
                .sort()
                .map((key) => ({
                    label: hmi.scenarioSet[key].scenarioSetName,
                    id: key,
                    child: hmi.scenarioSet[key].scenarios.map((item) => ({
                        label: item.scenarioName,
                        id: item.scenarioId,
                        content: item,
                    })),
                })),
        [hmi.scenarioSet],
    );

    // hmi.currentMap换成scenarios
    return <SourceList2<IScenarioInfo> activeId={hmi.currentScenarioId} onChange={onChange} items={items} />;
}

const ScenariosMemo = React.memo(Scenarios);

function Map() {
    const { t: translation } = useTranslation('modeSettings');
    const [hmi, dispatch] = usePickHmiStore();

    const { isMainConnected, mainApi } = useWebSocketServices();
    const onChange = useCallback(
        (mapId: any) => {
            if (isMainConnected) {
                dispatch(changeMap(mainApi, mapId, translation));
            }
        },
        [isMainConnected],
    );

    const items = useMemo(() => hmi.maps.map((item) => ({ id: item, label: item, content: item })), [hmi.maps]);

    return <SourceList1<string> activeId={hmi.currentMap} onChange={onChange} items={items} type='HDMap' />;
}

const MapMemo = React.memo(Map);

function Vehicle() {
    const [hmi, dispatch] = usePickHmiStore();
    const { isMainConnected, mainApi } = useWebSocketServices();
    const onChange = useCallback(
        (vehicle: any) => {
            if (isMainConnected) {
                dispatch(changeVehicle(mainApi, vehicle));
            }
        },
        [isMainConnected],
    );
    const items = useMemo(() => hmi.vehicles.map((item) => ({ id: item, label: item, content: item })), [hmi.vehicles]);
    return <SourceList1<string> activeId={hmi.currentVehicle} onChange={onChange} items={items} />;
}

const VehicleMemo = React.memo(Vehicle);

function V2X() {
    const onChange = (val: any) => {
        console.log(val);
    };
    return <SourceList1<string> activeId='' onChange={onChange} items={[]} />;
}

const V2XMemo = React.memo(V2X);

function EnviormentResources() {
    const { classes } = useStyle()();
    const { t } = useTranslation('modeSettings');
    const [{ activeEnviormentResourceTab }, dispatch] = useMenuStore();

    const [hmi] = usePickHmiStore();

    const EnviormentResourcesItems = useMemo(
        () =>
            ({
                [HMIModeOperation.PLAY_RECORDER]: [
                    {
                        key: ENUM_ENVIORMENT_MANAGER_TAB_KEY.RECORD,
                        label: t('records'),
                        children: <RecordersMemo />,
                    },
                    {
                        key: ENUM_ENVIORMENT_MANAGER_TAB_KEY.MAP,
                        label: t('HDMap'),
                        children: <MapMemo />,
                    },
                ],
                [HMIModeOperation.SIM_CONTROL]: [
                    {
                        key: ENUM_ENVIORMENT_MANAGER_TAB_KEY.MAP,
                        label: t('HDMap'),
                        children: <MapMemo />,
                    },
                ],
                [HMIModeOperation.SCENARIO]: [
                    {
                        key: ENUM_ENVIORMENT_MANAGER_TAB_KEY.SCENARIO,
                        label: t('scenario'),
                        children: <ScenariosMemo />,
                    },
                ],
            }[hmi.currentOperation]),
        [t, hmi.currentOperation],
    );

    const onChange = useCallback((key: string) => {
        dispatch(ChangeEnviormentResourcesAction(key as ENUM_ENVIORMENT_MANAGER_TAB_KEY));
    }, []);

    useEffect(() => {
        const hasItemHidden =
            EnviormentResourcesItems &&
            !EnviormentResourcesItems.some((item) => item.key === activeEnviormentResourceTab);
        if (hasItemHidden) {
            dispatch(ChangeEnviormentResourcesAction(EnviormentResourcesItems[0].key));
        }
    }, [activeEnviormentResourceTab, EnviormentResourcesItems]);

    if (!EnviormentResourcesItems) {
        return null;
    }

    const expendChild = (
        <Tabs
            activeKey={activeEnviormentResourceTab}
            rootClassName={classes.resource}
            items={EnviormentResourcesItems}
            onChange={onChange}
        />
    );

    return (
        <>
            <ModeSettingTitle expendChild={expendChild} title={t('enviormentResources')} />
            <Divider />
        </>
    );
}

const EnviormentResourcesMemo = React.memo(EnviormentResources);

function AdsResource() {
    const { classes } = useStyle()();
    const [{ activeAdsResourceTab }, dispatch] = useMenuStore();
    const { t } = useTranslation('modeSettings');
    const adsResourceItems = useMemo(
        () => [
            {
                key: ENUM_ADS_MANAGER_TAB_KEY.VEHICLE,
                label: t('vehicle'),
                children: <VehicleMemo />,
            },
        ],
        [t],
    );
    const onChange = useCallback((key: string) => {
        dispatch(ChangeAdsResourcesAction(key as ENUM_ADS_MANAGER_TAB_KEY));
    }, []);

    const expendChild = (
        <Tabs
            onChange={onChange}
            activeKey={activeAdsResourceTab}
            rootClassName={classes.resource}
            items={adsResourceItems}
        />
    );

    return <ModeSettingTitle expendChild={expendChild} title={t('adsResources')} />;
}

const AdsResourceMemo = React.memo(AdsResource);

function CurrentResource() {
    const [hmi] = usePickHmiStore();

    const { classes } = useCurrentResourceStyle();

    const { t } = useTranslation('modeSettings');

    const values = useMemo(
        () =>
            ['currentRecordId', 'currentScenarioName', 'currentMap', 'currentVehicle']
                .map((key: string) => hmi[key as keyof IInitState])
                .filter(Boolean),
        [hmi],
    );

    const expendChild = values.length ? (
        <>
            {values.map((item) => (
                <div title={item} className={classes['current-resource-item']} key={item}>
                    <span className={classes.name}>{item}</span>
                    <IconIcSucceed className='' />
                </div>
            ))}
        </>
    ) : (
        <div className={classes.empty}>
            <img alt='resource_empty' src={SourceEmptyImg} />
            <div>{t('empty')}</div>
        </div>
    );
    return (
        <>
            <ModeSettingTitle expendChild={expendChild} title={t('currentResource')} />
            <Divider />
        </>
    );
}

const CurrentResourceMemo = React.memo(CurrentResource);

function GudieContainer(props: PropsWithChildren<{ id: string }>) {
    const { classes } = useGuideContainerStyle();
    return (
        <div className={classes['guide-container']} id={props.id}>
            {props.children}
        </div>
    );
}

function ModeSetting() {
    const [, { bottomBarHeightString }] = useComponentDisplay();
    const height = useMemo(
        () => ({
            height: `calc(100vh - 78px - ${bottomBarHeightString})`,
        }),
        [bottomBarHeightString],
    );
    const { classes } = useStyle()(height);
    const { t } = useTranslation('modeSettings');
    return (
        <div className={classes['mode-setting']}>
            <MenuDrawerTitle title={t('modeSettings')} />
            <CustomScroll className={classes['mode-setting-container']}>
                {/* <Language /> */}
                <GudieContainer id='guide-modesettings-mode'>
                    <ModeMemo />
                </GudieContainer>
                <GudieContainer id='guide-modesettings-modules'>
                    <ModulesMemo />
                </GudieContainer>
                <GudieContainer id='guide-modesettings-operations'>
                    <OperationsMemo />
                </GudieContainer>
                <CurrentResourceMemo />
                <GudieContainer id='guide-modesettings-variable'>
                    <EnviormentResourcesMemo />
                </GudieContainer>
                <GudieContainer id='guide-modesettings-fixed'>
                    <AdsResourceMemo />
                </GudieContainer>
            </CustomScroll>
        </div>
    );
}

export default React.memo(ModeSetting);
