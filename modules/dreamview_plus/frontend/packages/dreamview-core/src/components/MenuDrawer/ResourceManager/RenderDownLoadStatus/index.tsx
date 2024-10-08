import React from 'react';
import { ENUM_DOWNLOAD_STATUS } from '@dreamview/dreamview-core/src/services/api/types';
import { useTranslation } from 'react-i18next';
import { IconPark } from '@dreamview/dreamview-ui';
import useStyle from './useStyle';

interface IRenderDownLoadStatus {
    status: ENUM_DOWNLOAD_STATUS;
    percentage: string;
}

function RenderDownLoadStatus(props: IRenderDownLoadStatus) {
    const { status, percentage } = props;
    const { classes, cx } = useStyle();
    const { t } = useTranslation('profileManagerRecords');

    if (status === ENUM_DOWNLOAD_STATUS.DOWNLOADED) {
        return (
            <div className={cx(classes.downloaded, classes['download-status'], 'download-active')}>
                <IconPark name='IcSucceed' />
                {t('downloaded')}
            </div>
        );
    }

    if (status === ENUM_DOWNLOAD_STATUS.DOWNLOADING) {
        return (
            <div className={cx(classes.downloading, classes['download-status'])}>
                <IconPark name='IcLoading' />
                {t('downloading')}
                &nbsp;
                {percentage}
            </div>
        );
    }

    if (status === ENUM_DOWNLOAD_STATUS.NOTDOWNLOAD) {
        return (
            <div className={cx(classes['download-status'])}>
                {' '}
                <IconPark name='IcToBeDownloaded' />
                {t('notDownload')}
            </div>
        );
    }

    if (status === ENUM_DOWNLOAD_STATUS.TOBEUPDATE) {
        return (
            <div className={cx(classes.tobeupdate, classes['download-status'])}>
                {' '}
                <IconPark name='IcToBeUpdated' />
                {t('tobeUpdate')}
            </div>
        );
    }

    if (status === ENUM_DOWNLOAD_STATUS.Fail) {
        return (
            <div className={cx(classes.downloadfail, classes['download-status'])}>
                {' '}
                <IconPark name='IcErrorMessage' />
                {t('downloadFail')}
            </div>
        );
    }

    return null;
}

export default React.memo(RenderDownLoadStatus);
