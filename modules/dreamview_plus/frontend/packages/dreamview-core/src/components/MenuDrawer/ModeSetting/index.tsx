/* eslint-disable react/jsx-no-bind */
/* eslint-disable jsx-a11y/label-has-associated-control */
import React, { PropsWithChildren, useCallback, useEffect, useMemo } from 'react';
import { Select, Switch, Tabs, IconIcSucceed } from '@dreamview/dreamview-ui';
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
} from '@dreamview/dreamview-core/src/store/HmiStore';
import SourceEmptyImg from '@dreamview/dreamview-core/src/assets/ic_default_page_no_data.png';
import throttle from 'lodash/throttle';
import Logger from '@dreamview/log';
import { useTranslation } from 'react-i18next';
import i18next from '@dreamview/dreamview-lang';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { useMainApi } from '@dreamview/dreamview-core/src/store/WebSocketManagerStore';
import { usePanelLayoutStore, updateByMode } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import MenuDrawerTitle from '../Common/MenuDrawerTitle';
import ModeSettingTitle from './Title';
import useStyle, { useCurrentResourceStyle, useGuideContainerStyle } from './useStyle';
import Divider from './Divider';
import Button from './Button';
import { SourceList1, SourceList2 } from './SourceList';
import CustomScroll from '../../CustomScroll';

const logger = Logger.getInstance('ModeSetting');

function Language() {
    const { t } = useTranslation('modeSettings');
    const options = [
        {
            label: '中文',
            value: 'zh',
        },
        {
            label: 'English',
            value: 'en',
        },
    ];
    const onChange = (val: string) => {
        if (val === 'zh') {
            i18next.changeLanguage('zh');
        } else {
            i18next.changeLanguage('en');
        }
    };

    return (
        <div>
            <ModeSettingTitle title={t('language')} />
            <Select onChange={onChange} options={options} placeholder='请选择' defaultValue={i18next.language} />
            <Divider />
        </div>
    );
}

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
    const options = useMemo(() => hmi.modes?.map((item) => ({ label: item, value: item })) || [], [hmi.modes]);

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
        () => hmi.operations?.map((item) => ({ label: item, value: item })) || [],
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
    const mainApi = useMainApi();
    const [hmi] = usePickHmiStore();
    const onChange = useCallback(
        throttle(() => {
            const checked = !!hmi.moduleStatus.get(moduleName);
            if (checked) {
                mainApi?.stopModule(moduleName);
            } else {
                mainApi?.startModule(moduleName);
            }
        }, THROTTLE_TIME),
        [mainApi, moduleName, hmi.moduleStatus.get(moduleName)],
    );

    return (
        <Switch
            onChange={onChange}
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

    // fixme 1期需求补充开关逻辑
    const onSetupAll = () => {
        // modeSetting -> modules模块
        // setup all
        if (!isMainConnected) return;
        mainApi?.setupAllModule();
    };

    const onResetAll = () => {
        // modeSetting -> modules模块
        // reset all
        if (!isMainConnected) return;
        mainApi?.resetAllModule();
    };

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
    const onChange = (scenario: any) => {
        dispatch(changeScenarios(scenario));
    };

    const items = useMemo(
        () =>
            Object.keys(hmi.scenarioSet)
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
    return <SourceList2<IScenarioInfo> activeId={hmi.currentMap} onChange={onChange} items={items} />;
}

const ScenariosMemo = React.memo(Scenarios);

function Map() {
    const [hmi, dispatch] = usePickHmiStore();

    const { isMainConnected, mainApi } = useWebSocketServices();
    const onChange = useCallback(
        (mapId: any) => {
            if (isMainConnected) {
                dispatch(changeMap(mainApi, mapId));
            }
        },
        [isMainConnected],
    );

    const items = useMemo(() => hmi.maps.map((item) => ({ id: item, label: item, content: item })), [hmi.maps]);

    return <SourceList1<string> activeId={hmi.currentMap} onChange={onChange} items={items} />;
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

function VariableResources() {
    const { classes } = useStyle()();
    const { t } = useTranslation('modeSettings');

    const [hmi] = usePickHmiStore();

    const variableResourcesItems = useMemo(
        () =>
            ({
                [HMIModeOperation.PLAY_RECORDER]: [
                    {
                        key: 'Recorders',
                        label: t('records'),
                        children: <RecordersMemo />,
                    },
                    {
                        key: 'Map',
                        label: t('HDMap'),
                        children: <MapMemo />,
                    },
                    // {
                    //     key: 'Scenarios',
                    //     label: 'Scenarios',
                    //     children: <Scenarios />,
                    // },
                ],
            }[hmi.currentOperation] || []),
        [t, hmi.currentOperation],
    );

    return (
        <>
            <ModeSettingTitle
                expendChild={<Tabs rootClassName={classes.resource} items={variableResourcesItems} />}
                title={t('variableResources')}
            />
            <Divider />
        </>
    );
}

const VariableResourcesMemo = React.memo(VariableResources);

function FixedResource() {
    const { classes } = useStyle()();
    const { t } = useTranslation('modeSettings');
    const fixedResourceItems = useMemo(
        () => [
            {
                key: 'Vehicle',
                label: t('vehicle'),
                children: <VehicleMemo />,
            },
            // {
            //     key: 'V2X',
            //     label: 'V2X',
            //     children: <V2X />,
            // },
        ],
        [t],
    );
    const expendChild = <Tabs rootClassName={classes.resource} items={fixedResourceItems} />;

    return <ModeSettingTitle expendChild={expendChild} title={t('fixedResources')} />;
}

const FixedResourceMemo = React.memo(FixedResource);

function CurrentResource() {
    const [hmi] = usePickHmiStore();

    const { classes } = useCurrentResourceStyle();

    const { t } = useTranslation('modeSettings');

    const values = useMemo(
        () =>
            ['currentRecordId', 'currentMap', 'currentVehicle']
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
    const [hmi] = usePickHmiStore();
    const isPlayerControlShow = hmiUtils.isPlayerControlShow(hmi);
    const height = useMemo(
        () => ({
            height: `calc(100vh - 78px - ${isPlayerControlShow ? 63 : 0}px)`,
        }),
        [isPlayerControlShow],
    );
    const { classes } = useStyle()(height);
    const { t } = useTranslation('modeSettings');
    return (
        <div className={classes['mode-setting']}>
            <MenuDrawerTitle title={t('modeSettings')} />
            <CustomScroll className={classes['mode-setting-container']} id='mode-setting-container-scroll'>
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
                    <VariableResourcesMemo />
                </GudieContainer>
                <GudieContainer id='guide-modesettings-fixed'>
                    <FixedResourceMemo />
                </GudieContainer>
            </CustomScroll>
        </div>
    );
}

export default React.memo(ModeSetting);
