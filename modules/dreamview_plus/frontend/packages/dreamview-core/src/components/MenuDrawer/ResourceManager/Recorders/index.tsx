import React, { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { TFunction } from 'i18next';
import { useTranslation } from 'react-i18next';
import { ENUM_DOWNLOAD_STATUS } from '@dreamview/dreamview-core/src/services/api/types';
import { DownloadRecord } from '@dreamview/dreamview-core/src/services/models/response-message.model';
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
    recordId: string;
    recordName: string;
    currentRecordId: string;
    refreshList: () => void;
    onUpdateDownloadProgress: (r: DownloadRecord) => void;
}

function Operations(props: IOperations) {
    const { refreshList, status, recordId, currentRecordId, onUpdateDownloadProgress, recordName } = props;
    const { isPluginConnected, pluginApi, mainApi } = useWebSocketServices();
    const usSubScribe = useRef<any>({
        do: () => true,
    });

    const onDelete = useCallback(() => {
        mainApi?.deleteRecord(recordName).then(() => {
            refreshList();
        });
    }, [mainApi]);

    const onDownload = useCallback(() => {
        // do download
        if (isPluginConnected) {
            let res: any = {};
            usSubScribe.current.do = pluginApi?.downloadRecord(recordId, recordId).subscribe(
                (r) => {
                    onUpdateDownloadProgress(r);
                    res = r;
                },
                () => {
                    onUpdateDownloadProgress({
                        ...res,
                        status: ENUM_DOWNLOAD_STATUS.Fail,
                    });
                },
            );
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
                console.log('pluginApi.downloadRecord usSubScribe failed');
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
            id={recordName}
            currentActiveId={currentRecordId}
        />
    );
}

const OperationsMemo = React.memo(Operations);

const getColumns = (
    t: TFunction<'profileManagerRecords'>,
    refreshList: () => void,
    updateDownloadProgress: (r: DownloadRecord) => void,
    currentRecordId: string,
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
                recordId={v.id}
                recordName={v.name}
                onUpdateDownloadProgress={updateDownloadProgress}
                currentRecordId={currentRecordId}
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

function Recorders() {
    const { isPluginConnected, pluginApi } = useWebSocketServices();
    const [hmi] = usePickHmiStore();
    const currentRecordId = hmi?.currentRecordId;
    const { t } = useTranslation('profileManagerRecords');
    const scrollHeight = useScrollHeight();

    const { data, setOriginData, refreshList } = useDataSource({
        apiConnected: isPluginConnected,
        api: () => pluginApi?.getRecordsList(),
        format,
        tabKey: ENUM_PROFILEMANAGER_TAB.Records,
    });

    const updateDownloadProgress = useCallback((r: DownloadRecord) => {
        setOriginData((prev) => {
            const recordId = r.resource_id;
            const record = prev[recordId];
            const percentage = Math.floor(r.percentage);
            if (r.status === ENUM_DOWNLOAD_STATUS.Fail) {
                record.status = ENUM_DOWNLOAD_STATUS.Fail;
            } else if (r.status === 'downloaded') {
                // if (percentage === 100) {
                record.status = ENUM_DOWNLOAD_STATUS.DOWNLOADED;
                record.percentage = percentage;
            } else {
                record.status = ENUM_DOWNLOAD_STATUS.DOWNLOADING;
                record.percentage = percentage;
            }
            return { ...prev };
        });
    }, []);

    const activeIndex = useMemo(
        () => data.findIndex((item) => item.name === currentRecordId) + 1,
        [data, currentRecordId],
    );
    const { classes } = useTableHover(activeIndex);

    const columns = useMemo(
        () => getColumns(t, refreshList, updateDownloadProgress, currentRecordId),
        [t, refreshList, updateDownloadProgress, currentRecordId],
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

export default React.memo(Recorders);
