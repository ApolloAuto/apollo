import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { TFunction } from 'i18next';
import { useTranslation } from 'react-i18next';
import { ENUM_DOWNLOAD_STATUS, DynamicModel } from '@dreamview/dreamview-core/src/services/api/types';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import { useTableHover } from '../useStyle';
import Table from '../Table';
import RenderDownLoadStatus from '../RenderDownLoadStatus';
import RenderName from '../RenderName';
import RenderOperation from '../RenderOperation';
import { useDataSource, useScrollHeight } from '../hoc';
import { ENUM_PROFILEMANAGER_TAB } from '../provider';
import Background from '../Background';

interface IOperations {
    status: ENUM_DOWNLOAD_STATUS;
    dynamicalName: string;
    currentDynamicModel: string;
    refreshList: () => void;
    onUpdateDownloadProgress: (r: DynamicModel) => void;
}

function Operations(props: IOperations) {
    const { refreshList, status, currentDynamicModel, onUpdateDownloadProgress, dynamicalName } = props;
    const { isPluginConnected, pluginApi, mainApi } = useWebSocketServices();
    const usSubScribe = useRef<any>({
        do: () => true,
    });

    const onDelete = useCallback(() => {
        mainApi?.deleteDynamicModel(dynamicalName).then(() => {
            refreshList();
        });
    }, [mainApi]);

    const onDownload = useCallback(() => {
        // do download
        if (isPluginConnected) {
            usSubScribe.current.do = pluginApi?.downloadDynamicModel(dynamicalName).subscribe((r) => {
                onUpdateDownloadProgress(r);
            });
        }
    }, [isPluginConnected, pluginApi]);

    const onRefresh = useCallback(() => false, []);

    const onCancelDownload = useCallback(() => {
        // do cancel download
        if (isPluginConnected) {
            // fixme: 暂无取消方法
        }
    }, [isPluginConnected]);

    useEffect(
        () => () => {
            try {
                usSubScribe.current.do();
            } catch (err) {
                console.log('pluginApi.DynamicModel usSubScribe failed');
            }
        },
        [],
    );

    return (
        <RenderOperation
            onDelete={onDelete}
            onRefresh={onRefresh}
            onDownload={onDownload}
            onCancelDownload={onCancelDownload}
            status={status}
            id={dynamicalName}
            currentActiveId={currentDynamicModel}
        />
    );
}

const OperationsMemo = React.memo(Operations);

const getColumns = (
    t: TFunction<'profileManagerRecords'>,
    refreshList: () => void,
    updateDownloadProgress: (r: DynamicModel) => void,
    currentDynamicModel: string,
) => [
    {
        title: t('titleName'),
        dataIndex: 'name',
        key: 'name',
        render: (v: string) => <RenderName name={v} />,
    },
    {
        title: t('titleType'),
        dataIndex: 'type',
        width: 250,
        key: 'type',
    },
    {
        title: t('titleState'),
        dataIndex: 'status',
        key: 'status',
        width: 240,
        // 当前单元格的值，当前行数据，当前行索引
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
                dynamicalName={v.name}
                onUpdateDownloadProgress={updateDownloadProgress}
                currentDynamicModel={currentDynamicModel}
            />
        ),
    },
];

const format = (v: any) =>
    Object.entries(v || {})
        .sort(([, a]: any, [, b]: any) => (a.name > b.name ? 1 : -1))
        .map(([key, value]: any) => ({
            percentage: value.percentage,
            status: value.status,
            name: value.name,
            // 暂时写死类型
            type: 'Official',
            id: key,
        }));

function Dynamical() {
    const { isPluginConnected, pluginApi } = useWebSocketServices();
    const [hmi] = usePickHmiStore();
    const currentDynamicModel = hmi?.currentDynamicModel;
    const { t } = useTranslation('profileManagerDynamical');
    const scrollHeight = useScrollHeight();

    const { data, setOriginData, refreshList } = useDataSource({
        apiConnected: isPluginConnected,
        api: () => pluginApi?.getDynamicModelList(),
        format,
        tabKey: ENUM_PROFILEMANAGER_TAB.Dynamical,
    });

    const updateDownloadProgress = useCallback((r: DynamicModel) => {
        setOriginData((prev) => {
            const dynamicalId = r.resource_id;
            const dynamical = prev[dynamicalId];
            const percentage = Math.floor(r.percentage);
            if (r.status === 'downloaded') {
                dynamical.status = ENUM_DOWNLOAD_STATUS.DOWNLOADED;
                dynamical.percentage = percentage;
            } else {
                dynamical.status = ENUM_DOWNLOAD_STATUS.DOWNLOADING;
                dynamical.percentage = percentage;
            }
            return { ...prev };
        });
    }, []);

    const activeIndex = useMemo(
        () => data.findIndex((item) => item.name === currentDynamicModel) + 1,
        [data, currentDynamicModel],
    );
    const { classes } = useTableHover(activeIndex);

    const columns = useMemo(
        () => getColumns(t, refreshList, updateDownloadProgress, currentDynamicModel),
        [t, refreshList, updateDownloadProgress, currentDynamicModel],
    );

    return (
        <Background>
            <Table
                className={classes['table-active']}
                scroll={{
                    y: scrollHeight,
                }}
                rowKey='id'
                columns={columns}
                data={data}
            />
        </Background>
    );
}

export default React.memo(Dynamical);
