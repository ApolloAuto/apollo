/* eslint-disable jsx-a11y/label-has-associated-control */
import React, { PropsWithChildren, useCallback, useEffect, useLayoutEffect, useMemo, useRef } from 'react';
import { IconPark, message, Select, SwitchLoading, Tabs, LightButton, useImagePrak } from '@dreamview/dreamview-ui';
import {
    changeDynamic,
    changeMap,
    changeMode,
    changeOperate,
    changeRecorder,
    changeRTKRecord,
    changeScenarios,
    changeVehicle,
    CURRENT_MODE,
    HMIModeOperation,
    IInitState,
    IScenarioInfo,
    RECORDER_LOAD_STATUS,
    THROTTLE_TIME,
    updateStatus,
    usePickHmiStore,
} from '@dreamview/dreamview-core/src/store/HmiStore';
import throttle from 'lodash/throttle';
import Logger from '@dreamview/log';
import { useTranslation } from 'react-i18next';
import useComponentDisplay from '@dreamview/dreamview-core/src/hooks/useComponentDisplay';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { useMainApi } from '@dreamview/dreamview-core/src/store/WebSocketManagerStore';
import {
    ChangeAdsResourcesAction,
    ChangeEnviormentResourcesAction,
} from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import {
    ENUM_ADS_MANAGER_TAB_KEY,
    ENUM_ENVIORMENT_MANAGER_TAB_KEY,
    useMenuStore,
} from '@dreamview/dreamview-core/src/store/MenuStore';
import { finalize, Subscription } from 'rxjs';
import MenuDrawerTitle from '../Common/MenuDrawerTitle';
import ModeSettingTitle from './Title';
import useStyle, { useCurrentResourceStyle, useGuideContainerStyle } from './useStyle';
import Divider from './Divider';
import { SourceList1, SourceList2 } from './SourceList';
import CustomScroll from '../../CustomScroll';
import { StreamDataNames } from '../../../services/api/types';

const logger = Logger.getInstance('ModeSetting');

const nillObj = {};

function Mode() {
    const { isMainConnected, mainApi } = useWebSocketServices();

    const [hmi, dispatch] = usePickHmiStore();
    const { classes } = useStyle(nillObj);

    const { t } = useTranslation('modeSettings');

    const onChange = useCallback(
        (v: CURRENT_MODE) => {
            if (isMainConnected) {
                dispatch(changeMode(mainApi, v));
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
            <Select onChange={onChange} value={hmi.currentMode} className={classes.space12px} options={options} />
            <Divider />
        </>
    );
}

const ModeMemo = React.memo(Mode);

function Operations() {
    const { isMainConnected, metadata, mainApi, streamApi } = useWebSocketServices();
    const [hmi, dispatch] = usePickHmiStore();
    const { classes } = useStyle(nillObj);

    const { t } = useTranslation('modeSettings');

    const simHMISubscriptionRef = useRef<Subscription | null>(null);

    useEffect(() => {
        if (hmi.currentOperation === HMIModeOperation.SCENARIO) {
            const initState: any = {};
            simHMISubscriptionRef.current = streamApi
                ?.subscribeToData(StreamDataNames.SIM_HMI_STATUS)
                .pipe(
                    finalize(() => {
                        dispatch(
                            updateStatus({
                                ...(initState as any),
                                frontendIsFromSimHmi: true,
                            } as any),
                        );
                    }),
                )
                .subscribe((data) => {
                    Object.keys(data).forEach((key) => {
                        initState[key] = undefined;
                    });
                    dispatch(
                        updateStatus({
                            ...(data as any),
                            frontendIsFromSimHmi: true,
                        }),
                    );
                });
        }

        return () => {
            simHMISubscriptionRef.current?.unsubscribe();
        };
    }, [dispatch, hmi.currentOperation, streamApi]);

    const options = useMemo(
        () => hmi.operations?.map((item) => ({ label: item, value: `${item}` })) || [],
        [hmi.operations],
    );

    const onChange = useCallback(
        (operate: string) => {
            if (
                hmi.currentOperation === HMIModeOperation.AUTO_DRIVE ||
                hmi.currentOperation === HMIModeOperation.WAYPOINT_FOLLOW
            ) {
                mainApi.stopAutoDrive();
            }
            if (hmi.currentOperation === HMIModeOperation.SCENARIO) {
                simHMISubscriptionRef.current?.unsubscribe();
            }
            dispatch(changeOperate(mainApi, operate));
        },
        [isMainConnected, hmi.currentOperation],
    );

    return (
        <>
            <ModeSettingTitle title={t('operations')} />
            <Select onChange={onChange} value={hmi.currentOperation} className={classes.space12px} options={options} />
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
    const moduleStatus = hmi.modules.get(moduleName);
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
            return (
                <IconPark
                    name='IcLoading'
                    spin
                    className='switch-inner-loading-icon'
                    onPointerOverCapture={undefined}
                    onPointerOutCapture={undefined}
                />
            );
        }
        return null;
    }, [mainApi, moduleName, isModuleLocked]);

    const loadingChild = useMemo(getLoadingIcon, [getLoadingIcon]);

    return (
        <SwitchLoading
            onChange={onChange}
            checkedChildren={loadingChild}
            unCheckedChildren={loadingChild}
            checked={!!hmi.modules.get(moduleName)}
        />
    );
}

const CustomSwitchMemo = React.memo(CustomSwitch);

function Modules() {
    const { classes } = useStyle(nillObj);

    const { isMainConnected, mainApi } = useWebSocketServices();
    const [hmi] = usePickHmiStore();
    const { t } = useTranslation('modeSettings');

    const modules = Array.from(hmi.modules.keys());

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
                <LightButton onClick={onSetupAll} width={153}>
                    {t('setupAll')}
                </LightButton>
                <LightButton onClick={onResetAll} width={153}>
                    {t('resetAll')}
                </LightButton>
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
                    // 预加载状态
                    preLoad: hmi.records[item].loadRecordStatus,
                }))
                .sort((a, b) => a.id.localeCompare(b.id)),
        [hmi.records],
    );
    const onChange = useCallback(
        (recorderId: string, currentPreLoadStatus?: RECORDER_LOAD_STATUS) => {
            if (currentPreLoadStatus === RECORDER_LOAD_STATUS.NOT_LOAD) {
                mainApi.loadRecord(recorderId).then(() => logger.debug(`loadRecord ${recorderId} success`));
            }
            if (currentPreLoadStatus === RECORDER_LOAD_STATUS.LOADING) {
                return;
            }
            if (isMainConnected && currentPreLoadStatus === RECORDER_LOAD_STATUS.LOADED) {
                dispatch(changeRecorder(mainApi, recorderId));
            }
            logger.error(`recorderId: ${recorderId} currentPreLoadStatus: ${currentPreLoadStatus}`);
        },
        [isMainConnected],
    );

    return <SourceList1<string> activeId={hmi?.currentRecordId} onChange={onChange} items={items} />;
}

const RecordersMemo = React.memo(Recorders);

function RTKRecorders() {
    const { isMainConnected, mainApi } = useWebSocketServices();
    const [hmi, dispatch] = usePickHmiStore();
    const items = useMemo(
        () =>
            hmi.rtkRecords
                .map((item) => ({
                    id: item,
                    label: item,
                    content: item,
                }))
                .sort((a, b) => a.id.localeCompare(b.id)),
        [hmi.rtkRecords],
    );

    const onChange = useCallback(
        (recorderId: string) => {
            if (isMainConnected) {
                dispatch(changeRTKRecord(mainApi, recorderId));
            }
        },
        [isMainConnected],
    );

    return <SourceList1<string> activeId={hmi?.currentRtkRecordId} onChange={onChange} items={items} />;
}

const RTKRecordersMemo = React.memo(RTKRecorders);

function Scenarios() {
    const [hmi, dispatch] = usePickHmiStore();
    const { otherApi, mainApi } = useWebSocketServices();

    useLayoutEffect(() => {
        otherApi.loadScenarios();
    }, []);

    const onChange = useCallback((scenariosSet: any, scenarios: any) => {
        dispatch(
            changeScenarios(otherApi, mainApi, {
                scenariosSetId: scenariosSet.id,
                scenarioId: scenarios.scenarioId,
            }),
        );
    }, []);

    const items = useMemo(
        () =>
            Object.keys(hmi.scenarioSet || {})
                .sort()
                .map((key) => ({
                    label: hmi.scenarioSet[key].scenarioSetName,
                    id: key,
                    child:
                        hmi.scenarioSet[key].scenarios?.map((item) => ({
                            label: item.scenarioName,
                            id: item.scenarioId,
                            content: item,
                        })) || [],
                })),
        [hmi.scenarioSet],
    );

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

    const items = useMemo(() => {
        const maps = hmi.maps.map((item) => ({ id: item, label: item, content: item }));
        return maps.sort((prev, next) => (prev.id > next.id ? 1 : -1));
    }, [hmi.maps]);

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

function DynamicModels() {
    const [hmi, dispatch] = usePickHmiStore();
    const { mainApi } = useWebSocketServices();
    const items = useMemo(
        () =>
            hmi.dynamicModels
                .map((item) => ({
                    id: item,
                    label: item,
                    content: item,
                }))
                .sort((a, b) => a.id.localeCompare(b.id)),
        [hmi.dynamicModels],
    );

    const onChange = useCallback((dynamicId: string) => {
        dispatch(changeDynamic(mainApi, dynamicId));
    }, []);

    return <SourceList1<string> activeId={hmi.currentDynamicModel} onChange={onChange} items={items} />;
}

const DynamicModelsMemo = React.memo(DynamicModels);

function EnviormentResources() {
    const { classes } = useStyle(nillObj);
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
                [HMIModeOperation.AUTO_DRIVE]: [
                    {
                        key: ENUM_ENVIORMENT_MANAGER_TAB_KEY.MAP,
                        label: t('HDMap'),
                        children: <MapMemo />,
                    },
                ],
                [HMIModeOperation.WAYPOINT_FOLLOW]: [
                    {
                        key: ENUM_ENVIORMENT_MANAGER_TAB_KEY.RECORD,
                        label: t('RTKRecords'),
                        children: <RTKRecordersMemo />,
                    },
                    {
                        key: ENUM_ENVIORMENT_MANAGER_TAB_KEY.MAP,
                        label: t('HDMap'),
                        children: <MapMemo />,
                    },
                ],
                [HMIModeOperation.None]: [],
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
    const { classes } = useStyle(nillObj);
    const [{ activeAdsResourceTab }, dispatch] = useMenuStore();
    const { t } = useTranslation('modeSettings');
    const [{ isDynamicalModelsShow }] = useComponentDisplay();
    const adsResourceItems = useMemo(
        () =>
            [
                {
                    key: ENUM_ADS_MANAGER_TAB_KEY.VEHICLE,
                    label: t('vehicle'),
                    children: <VehicleMemo />,
                },
                isDynamicalModelsShow && {
                    key: ENUM_ADS_MANAGER_TAB_KEY.DYNAMIC,
                    label: t('dynamic'),
                    children: <DynamicModelsMemo />,
                },
            ].filter(Boolean),
        [t, isDynamicalModelsShow],
    );
    useEffect(() => {
        const hasItemHidden = adsResourceItems && !adsResourceItems.some((item) => item.key === activeAdsResourceTab);
        if (hasItemHidden) {
            dispatch(ChangeAdsResourcesAction(adsResourceItems[0].key));
        }
    }, [adsResourceItems]);
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

    const SourceEmptyImg = useImagePrak('ic_default_page_no_data');
    const values = useMemo(
        () =>
            [
                'currentRecordId',
                'currentScenarioName',
                'currentMap',
                'currentVehicle',
                'currentDynamicModel',
                'currentRtkRecordId',
            ]
                .map((key: string) => hmi[key as keyof IInitState])
                .filter(Boolean),
        [hmi],
    );

    const expendChild = values.length ? (
        <>
            {values.map((item) => (
                <div title={item} className={classes['current-resource-item']} key={item}>
                    <span className={classes.name}>{item}</span>
                    <IconPark
                        name='IcSucceed'
                        className=''
                        onPointerOverCapture={undefined}
                        onPointerOutCapture={undefined}
                    />
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
    const [hmi] = usePickHmiStore();
    // console.log('hmi', hmi);
    const [, { bottomBarHeightString }] = useComponentDisplay();
    const styles = useMemo(
        () => ({
            height: `calc(100vh - 78px - ${bottomBarHeightString})`,
        }),
        [bottomBarHeightString],
    );
    const { classes } = useStyle(styles);
    const { t } = useTranslation('modeSettings');

    const modeDisplay = () => hmi.modules?.size > 0;

    const operationsDisplay = () => hmi.currentOperation !== HMIModeOperation.None;

    const curResourceDisplay = () => hmi.currentOperation !== HMIModeOperation.None;

    const envResourceDisplay = () => hmi.currentOperation !== HMIModeOperation.None;

    const adsResourceDisplay = () => hmi.currentOperation !== HMIModeOperation.None;

    return (
        <div className={classes['mode-setting']}>
            <MenuDrawerTitle title={t('modeSettings')} />
            <CustomScroll className={classes['mode-setting-container']}>
                {/* <Language /> */}
                <GudieContainer id='guide-modesettings-mode'>
                    <ModeMemo />
                </GudieContainer>
                {modeDisplay() && (
                    <GudieContainer id='guide-modesettings-modules'>
                        <ModulesMemo />
                    </GudieContainer>
                )}
                {operationsDisplay() && (
                    <GudieContainer id='guide-modesettings-operations'>
                        <OperationsMemo />
                    </GudieContainer>
                )}
                {curResourceDisplay() && <CurrentResourceMemo />}
                {envResourceDisplay() && (
                    <GudieContainer id='guide-modesettings-variable'>
                        <EnviormentResourcesMemo />
                    </GudieContainer>
                )}
                {adsResourceDisplay() && (
                    <GudieContainer id='guide-modesettings-fixed'>
                        <AdsResourceMemo />
                    </GudieContainer>
                )}
            </CustomScroll>
        </div>
    );
}

export default React.memo(ModeSetting);
