import React from 'react';
import { ENUM_DOWNLOAD_STATUS } from '@dreamview/dreamview-core/src/services/api/types';
import { IconPark, Popover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';

interface IRenderDownLoadStatus {
    status: ENUM_DOWNLOAD_STATUS;
    id: string;
    currentActiveId: string;
    onDelete: () => void;
    onCancelDownload: () => void;
    onDownload: () => void;
    onRefresh: () => void;
    deleteButton?: boolean;
}

function RenderOperation(props: IRenderDownLoadStatus) {
    const {
        status,
        id,
        currentActiveId,
        onDelete,
        onDownload,
        onCancelDownload,
        deleteButton = true,
        onRefresh,
    } = props;
    const { classes } = useStyle();
    const isInUsed = id === currentActiveId;

    const { t } = useTranslation('profileManagerOperate');

    const icInUse = <IconPark name='IcSucceed' className={classes['source-operate-icon']} />;

    const icDelete = deleteButton ? (
        <Popover trigger='hover' content={t('delete')}>
            <IconPark name='IcDelete' onClick={onDelete} className={classes['source-operate-icon']} />
        </Popover>
    ) : null;

    const icCancel = (
        <Popover trigger='hover' content={t('cancel')}>
            <IconPark
                name='IcDownloadingCancel'
                onClick={onCancelDownload}
                className={classes['source-operate-icon']}
            />
        </Popover>
    );
    const icDownload = (
        <Popover trigger='hover' content={t('download')}>
            <IconPark name='IcDownloading' onClick={onDownload} className={classes['source-operate-icon']} />
        </Popover>
    );
    const icRefresh = (
        <Popover trigger='hover' content={t('update')}>
            <IconPark name='IcUseRetry' onClick={onRefresh} className={classes['source-operate-icon']} />
        </Popover>
    );
    const icRetry = (
        <Popover trigger='hover' content={t('retry')}>
            <IconPark name='IcUseRetry' onClick={onDownload} className={classes['source-operate-icon']} />
        </Popover>
    );

    if (isInUsed) {
        return <div className={classes['source-operate']}>{icInUse}</div>;
    }

    if (status === ENUM_DOWNLOAD_STATUS.DOWNLOADED) {
        return <div className={classes['source-operate']}>{icDelete || '--'}</div>;
    }

    if (status === ENUM_DOWNLOAD_STATUS.DOWNLOADING) {
        return <div className={classes['source-operate']}>{icCancel}</div>;
    }

    if (status === ENUM_DOWNLOAD_STATUS.NOTDOWNLOAD) {
        return <div className={classes['source-operate']}>{icDownload}</div>;
    }

    if (status === ENUM_DOWNLOAD_STATUS.Fail) {
        return <div className={classes['source-operate']}>{icRetry}</div>;
    }

    if (status === ENUM_DOWNLOAD_STATUS.TOBEUPDATE) {
        return <div className={classes['source-operate']}>{icRefresh}</div>;
    }
}

export default React.memo(RenderOperation);
