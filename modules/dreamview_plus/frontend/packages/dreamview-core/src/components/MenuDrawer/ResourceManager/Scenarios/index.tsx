/* eslint-disable camelcase */
import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { TFunction } from 'i18next';
import { useTranslation } from 'react-i18next';
import { ENUM_DOWNLOAD_STATUS, ScenarioSet, ScenarioSetRecord } from '@dreamview/dreamview-core/src/services/api/types';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import { OperatePopover, message } from '@dreamview/dreamview-ui';
import { useUserInfoStore } from '@dreamview/dreamview-core/src/store/UserInfoStore';
import Table from '../Table';
import RenderDownLoadStatus from '../RenderDownLoadStatus';
import RenderName from '../RenderName';
import RenderOperation from '../RenderOperation';
import { useTableHover, useTitleTrangle } from '../useStyle';
import { useDataSource, useScrollHeight } from '../hoc';
import { ENUM_PROFILEMANAGER_TAB } from '../provider';
import Background from '../Background';

type IScenarioSet = ScenarioSet & { frontpercentage: string };

interface IOperations {
    status: ENUM_DOWNLOAD_STATUS;
    id: string;
    recordName: string;
    is_classic: boolean;
    currentScenarioId: string;
    refreshList: () => void;
    onUpdateDownloadProgress: (r: IScenarioSet) => void;
    onDelete: (r: string) => any;
}

function messageError(err: any) {
    const code = err.data.info.code;
    if (code === 50008 || code === 35004) {
        return;
    }
    message({
        type: 'error',
        content: err.data.info.message,
    });
}

function Operations(props: IOperations) {
    const { onDelete: propOnDelete, status, id, is_classic, currentScenarioId, onUpdateDownloadProgress } = props;
    const { isPluginConnected, pluginApi } = useWebSocketServices();
    const usSubScribe = useRef<any>({
        do: () => true,
    });

    const onDelete = useCallback(() => {
        propOnDelete(id);
    }, []);

    const onDownload = useCallback(() => {
        // do download
        if (isPluginConnected) {
            usSubScribe.current.do = pluginApi?.downloadScenarioSet(id, is_classic, id).subscribe((r) => {
                onUpdateDownloadProgress(r as any);
            });
        }
    }, [isPluginConnected, pluginApi]);

    const onCancelDownload = useCallback(() => false, []);

    useEffect(
        () => () => {
            try {
                usSubScribe.current.do();
            } catch (err) {
                console.log('pluginApi.downloadScenarios usSubScribe failed');
            }
        },
        [],
    );

    return (
        <RenderOperation
            onDelete={onDelete}
            onDownload={onDownload}
            onRefresh={onDownload}
            onCancelDownload={onCancelDownload}
            status={status}
            id={id}
            currentActiveId={currentScenarioId}
        />
    );
}

const OperationsMemo = React.memo(Operations);

const getColumns = (
    operateNode: any,
    cs: any,
    t: TFunction<'profileScenarios'>,
    refreshList: () => void,
    updateDownloadProgress: (r: IScenarioSet) => void,
    currentScenarioId: string,
    onDelete: (r: string) => any,
) => [
    {
        title: t('titleName'),
        dataIndex: 'name',
        key: 'name',
        render: (v: string) => <RenderName name={v} />,
    },
    {
        title: (
            <OperatePopover placement='bottomRight' trigger='hover' content={operateNode}>
                <span>
                    <span className={cs['title-operate']}>
                        {t('titleType')}
                        <div className={cs['title-triangle']} />
                    </span>
                </span>
            </OperatePopover>
        ),
        dataIndex: 'type',
        width: 250,
        key: 'type',
    },
    {
        title: t('titleState'),
        dataIndex: 'status',
        key: 'status',
        width: 240,
        render: (v: ENUM_DOWNLOAD_STATUS, item: any) => (
            <RenderDownLoadStatus percentage={`${item.percentage}%`} status={v} />
        ),
    },
    {
        title: t('titleOperate'),
        key: 'address',
        width: 200,
        render: (v: any) => (
            <OperationsMemo
                refreshList={refreshList}
                status={v.status}
                id={v.id}
                is_classic={v.is_classic}
                onDelete={onDelete}
                recordName={v.name}
                onUpdateDownloadProgress={updateDownloadProgress}
                currentScenarioId={currentScenarioId}
            />
        ),
    },
];

function getType(scenario: any, t: any) {
    if (scenario.category === 'public') {
        return t('system');
    }
    if (scenario.category === 'subscriber' && scenario.public) {
        return t('team');
    }
    return t('personal');
}

enum FILTER_TYPE {
    ALL = 'all',
    TEAM = 'team',
    PERSONAL = 'personal',
    SYSTEM = 'system',
}

function Scenarios() {
    const { isPluginConnected, pluginApi, isMainConnected, mainApi, otherApi } = useWebSocketServices();
    const [hmi] = usePickHmiStore();
    const [{ account }] = useUserInfoStore();

    const currentScenarioSetId = hmi?.currentScenarioSetId;
    const { t } = useTranslation('profileManagerScenarios');
    const scrollHeight = useScrollHeight();

    const loadScenarios = useCallback(() => {
        otherApi.loadScenarios();
    }, []);

    const { data, setOriginData, refreshList } = useDataSource<ScenarioSetRecord>({
        apiConnected: isPluginConnected,
        api: () => pluginApi?.getScenarioSetList(),
        format(v) {
            return Object.entries(v || {})
                .sort(([, a]: any, [, b]: any) => (a.name > b.name ? 1 : -1))
                .map(([key, value]: any) => ({
                    percentage: value.percentage,
                    status: value.status,
                    name: value.name,
                    public: value.public,
                    category: value.category,
                    is_classic: value.is_classic,
                    // 暂时写死类型
                    type: getType(value, t),
                    id: key,
                }));
        },
        tabKey: ENUM_PROFILEMANAGER_TAB.Scenarios,
    });

    useEffect(() => {
        refreshList();
    }, [account?.subscriber?.subscriberId]);

    const [filterType, setFilterType] = useState(FILTER_TYPE.ALL);

    const myData = useMemo(() => {
        if (filterType === FILTER_TYPE.ALL) {
            return data;
        }
        if (filterType === FILTER_TYPE.SYSTEM) {
            return data.filter((item) => item.category === 'public');
        }
        if (filterType === FILTER_TYPE.TEAM) {
            return data.filter((item) => item.category === 'subscriber' && item.public);
        }
        return data
            .filter((item) => !(item.category === 'public'))
            .filter((item) => !(item.category === 'subscriber' && item.public));
    }, [data, filterType]);

    const onDelete = useCallback(
        (type: string) => {
            if (isMainConnected) {
                return mainApi.deleteScenarioSet(type).then(() => {
                    refreshList();
                    loadScenarios();
                });
            }
            return Promise.reject();
        },
        [isMainConnected, loadScenarios],
    );

    const updateDownloadProgress = useCallback(
        (r: IScenarioSet) => {
            setOriginData((prev) => {
                const id = r.resource_id;
                const record = prev[id];
                const percentage = Math.floor(r.percentage);
                if (r.status === 'downloaded') {
                    record.status = ENUM_DOWNLOAD_STATUS.DOWNLOADED;
                    record.percentage = 100;
                    loadScenarios();
                } else {
                    record.status = ENUM_DOWNLOAD_STATUS.DOWNLOADING;
                    record.percentage = percentage;
                }
                return { ...prev };
            });
        },
        [loadScenarios],
    );

    const { classes: cs, cx } = useTitleTrangle();

    const operates = [
        {
            key: FILTER_TYPE.ALL,
            handler() {
                setFilterType(FILTER_TYPE.ALL);
            },
        },
        {
            key: FILTER_TYPE.SYSTEM,
            handler() {
                setFilterType(FILTER_TYPE.SYSTEM);
            },
        },
    ];

    if (account?.subscriber?.subscriberId) {
        operates.push(
            {
                key: FILTER_TYPE.TEAM,
                handler() {
                    setFilterType(FILTER_TYPE.TEAM);
                },
            },
            {
                key: FILTER_TYPE.PERSONAL,
                handler() {
                    setFilterType(FILTER_TYPE.PERSONAL);
                },
            },
        );
    } else {
        operates.push({
            key: FILTER_TYPE.PERSONAL,
            handler() {
                setFilterType(FILTER_TYPE.PERSONAL);
            },
        });
    }

    const operateNode = (
        <div className={cs['operate-container']}>
            {operates.map((item) => (
                <div
                    className={cx(cs['operate-item'], { [cs['operate-item-active']]: item.key === filterType })}
                    key={item.key}
                    onClick={item.handler}
                >
                    {t(item.key)}
                </div>
            ))}
        </div>
    );

    const columns = useMemo(
        () => getColumns(operateNode, cs, t, refreshList, updateDownloadProgress, currentScenarioSetId, onDelete),
        [t, refreshList, updateDownloadProgress, currentScenarioSetId, onDelete, filterType],
    );

    const activeIndex = useMemo(
        () => data.findIndex((item) => item.id === currentScenarioSetId) + 1,
        [data, currentScenarioSetId],
    );

    const { classes } = useTableHover(activeIndex);

    return (
        <Background>
            <Table
                className={classes['table-active']}
                scroll={{
                    y: scrollHeight,
                }}
                rowKey='id'
                columns={columns}
                data={myData}
            />
        </Background>
    );
}

export default React.memo(Scenarios);
