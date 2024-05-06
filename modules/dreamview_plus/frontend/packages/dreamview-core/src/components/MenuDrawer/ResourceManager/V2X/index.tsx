import React, { useCallback, useMemo } from 'react';
import { TFunction } from 'i18next';
import { useTranslation } from 'react-i18next';
import { V2xInfoRecord, ENUM_DOWNLOAD_STATUS } from '@dreamview/dreamview-core/src/services/api/types';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import Table from '../Table';
import RenderName from '../RenderName';
import { useDataSource, useScrollHeight } from '../hoc';
import { ENUM_PROFILEMANAGER_TAB } from '../provider';
import Operation from './Operation';
import RenderDownLoadStatus from '../RenderDownLoadStatus';
import Background from '../Background';

const getColumns = (
    t: TFunction<'profileManagerV2X'>,
    onReset: (id: string) => Promise<any>,
    onRefresh: (id: string) => Promise<any>,
    currentV2XId: string,
    onUpload: (id: string) => Promise<any>,
    onDelete: (id: string) => Promise<any>,
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
        render: (v: ENUM_DOWNLOAD_STATUS, item: any) => (
            <RenderDownLoadStatus percentage={`${item.percentage}%`} status={v} />
        ),
    },
    {
        title: t('titleOperate'),
        key: 'address',
        width: 200,
        render: (v: any) => (
            <Operation
                onUpload={onUpload}
                status={v.status}
                name={v.deleteName}
                onReset={onReset}
                onRefresh={onRefresh}
                onDelete={onDelete}
                id={v.id}
                currentActiveId={currentV2XId}
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
            name: value.obu_in,
            // 暂时写死类型
            type: value.type,
            id: key,
            deleteName: value.vehicle_name,
        }));

function Vehicle() {
    const { isPluginConnected, pluginApi, isMainConnected, mainApi } = useWebSocketServices();
    const [hmi] = usePickHmiStore();

    const currentV2XId = hmi?.currentVehicle;
    const { t } = useTranslation('profileManagerV2X');
    const scrollHeight = useScrollHeight();
    const { data, refreshList } = useDataSource<V2xInfoRecord>({
        apiConnected: isPluginConnected,
        api: () => pluginApi?.getV2xInfo(),
        format,
        tabKey: ENUM_PROFILEMANAGER_TAB.V2X,
    });

    const onReset = useCallback(
        (id: string) => {
            if (isPluginConnected) {
                return pluginApi.resetV2xConfig(id).then(() => {
                    refreshList();
                });
            }
            return Promise.reject();
        },
        [isPluginConnected],
    );

    const onRefresh = useCallback(
        (id: string) => {
            if (isPluginConnected) {
                return pluginApi.refreshV2xConf(id).then(() => {
                    refreshList();
                });
            }
            return Promise.reject();
        },
        [isPluginConnected],
    );

    const onUpload = useCallback(
        (id: string) => {
            if (isPluginConnected) {
                return pluginApi.uploadV2xConf(id).then(() => {
                    refreshList();
                });
            }
            return Promise.reject();
        },
        [isPluginConnected],
    );

    const onDelete = useCallback(
        (type: string) => {
            if (isMainConnected) {
                return mainApi.deleteV2XConfig(type).then(() => {
                    refreshList();
                });
            }
            return Promise.reject();
        },
        [isMainConnected],
    );

    const columns = useMemo(
        () => getColumns(t, onReset, onRefresh, currentV2XId, onUpload, onDelete),
        [t, onReset, onRefresh, currentV2XId, onUpload, onDelete],
    );

    return (
        <Background>
            <Table
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

export default React.memo(Vehicle);
