import React, { useCallback, useMemo } from 'react';
import { TFunction } from 'i18next';
import { useTranslation } from 'react-i18next';
import { VehicleInfoRecord, ENUM_DOWNLOAD_STATUS } from '@dreamview/dreamview-core/src/services/api/types';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import { usePickHmiStore } from '@dreamview/dreamview-core/src/store/HmiStore';
import Table from '../Table';
import RenderName from '../RenderName';
import { useDataSource, useScrollHeight } from '../hoc';
import { ENUM_PROFILEMANAGER_TAB } from '../provider';
import Operation from './Operation';
import RenderDownLoadStatus from '../RenderDownLoadStatus';
import Background from '../Background';

const OperationsMemo = React.memo(Operation);

const getColumns = (
    t: TFunction<'profileManagerVehicle'>,
    onReset: (id: string) => Promise<any>,
    onRefresh: (id: string) => Promise<any>,
    currentVehicleId: string,
    onUpload: (id: string) => Promise<any>,
    onDelete: (type: string) => Promise<any>,
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
            <OperationsMemo
                onUpload={onUpload}
                status={v.status}
                onReset={onReset}
                onDelete={onDelete}
                onRefresh={onRefresh}
                id={v.id}
                type={v.type}
                currentActiveId={currentVehicleId}
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
            name: value.vin,
            // 暂时写死类型
            type: `${value?.vtype[0]?.toUpperCase()}${value.vtype
                .slice(1)
                .replace(/_([a-z])/g, (match: string, $1: string) => ` ${$1.toUpperCase()}`)}`,
            id: value.vehicle_id,
        }));

function Vehicle() {
    const { isPluginConnected, pluginApi, mainApi, isMainConnected } = useWebSocketServices();
    const [hmi] = usePickHmiStore();

    const currentVehicleId = hmi?.currentVehicle;
    const { t } = useTranslation('profileManagerVehicle');
    const scrollHeight = useScrollHeight();

    const { data, refreshList } = useDataSource<VehicleInfoRecord>({
        apiConnected: isPluginConnected,
        api: () => pluginApi?.getVehicleInfo(),
        format,
        tabKey: ENUM_PROFILEMANAGER_TAB.Vehicle,
    });

    const onReset = useCallback(
        (id: string) => {
            if (isPluginConnected) {
                return pluginApi.resetVehicleConfig(id).then(() => {
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
                return pluginApi.refreshVehicleConfig(id).then(() => {
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
                return pluginApi.uploadVehicleConfig(id).then(() => {
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
                return mainApi.deleteVehicleConfig(type).then(() => {
                    refreshList();
                });
            }
            return Promise.reject();
        },
        [isMainConnected],
    );

    const columns = useMemo(
        () => getColumns(t, onReset, onRefresh, currentVehicleId, onUpload, onDelete),
        [t, onReset, onRefresh, currentVehicleId, onUpload, onDelete],
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
