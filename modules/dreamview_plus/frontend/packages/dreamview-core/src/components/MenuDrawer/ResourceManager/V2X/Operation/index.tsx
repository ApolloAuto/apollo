import React, { useCallback, useState } from 'react';
import { useTranslation } from 'react-i18next';
import { IconPark, Popover } from '@dreamview/dreamview-ui';
import { ENUM_DOWNLOAD_STATUS } from '@dreamview/dreamview-core/src/services/api/types';
import useStyle from './useStyle';

interface IRenderDownLoadStatus {
    id: string;
    currentActiveId: string;
    name: string;
    status: ENUM_DOWNLOAD_STATUS;
    onReset: (id: string) => Promise<any>;
    onRefresh: (id: string) => Promise<any>;
    onUpload?: (id: string) => Promise<any>;
    onDelete?: (name: string) => Promise<any>;
}

function useDisabled(): [boolean, (func: () => Promise<any>) => void] {
    const [disabled, setDisabled] = useState(false);

    const onDo = (func: () => Promise<any>) => {
        setDisabled(true);

        if (disabled) return;

        func()
            .then(() => {
                setDisabled(false);
            })
            .catch(() => {
                setDisabled(false);
            });
    };

    return [disabled, onDo];
}

function RenderOperation(props: IRenderDownLoadStatus) {
    const {
        id,
        currentActiveId,
        status,
        name,
        onReset: propOnReset,
        onRefresh: propOnRefresh,
        onUpload: propOnUpload = () => Promise.resolve(),
        onDelete: propOnDelete = () => Promise.resolve(),
    } = props;
    const { t } = useTranslation('profileManagerOperate');
    const { classes, cx } = useStyle();
    const isInUsed = id === currentActiveId;

    const [disabled, onDo] = useDisabled();

    const onRefresh = useCallback(() => {
        onDo(() => propOnRefresh(id));
    }, []);

    const onReset = useCallback(() => {
        onDo(() => propOnReset(id));
    }, []);

    const onUpload = useCallback(() => {
        onDo(() => propOnUpload(id));
    }, []);

    const onDelete = useCallback(() => {
        onDo(() => propOnDelete(name));
    }, []);

    const icUpload = (
        <Popover className={classes.font18} trigger='hover' content={t('upload')}>
            <span onClick={onUpload}>
                <IconPark name='Upload' />
            </span>
        </Popover>
    );

    const icDelete = (
        <Popover trigger='hover' content={t('delete')}>
            <span onClick={onDelete}>
                <IconPark name='IcDelete' />
            </span>
        </Popover>
    );

    const icUpdate = (
        <Popover trigger='hover' content={t('download')}>
            <span onClick={onRefresh}>
                <IconPark name='IcDownloading' />
            </span>
        </Popover>
    );

    const icReset = (
        <Popover trigger='hover' content={t('reset')}>
            <span className={classes.retry} onClick={onReset}>
                <IconPark name='IcUseRetry' />
            </span>
        </Popover>
    );

    if (isInUsed) {
        return (
            <div>
                <IconPark name='IcSucceed' className={classes['source-operate-icon']} />
            </div>
        );
    }

    if (status === ENUM_DOWNLOAD_STATUS.DOWNLOADED) {
        return (
            <div className={cx(classes['source-operate'], { disabled })}>
                {icUpload}
                {icDelete}
            </div>
        );
    }

    if (status === ENUM_DOWNLOAD_STATUS.NOTDOWNLOAD) {
        return (
            <div className={cx(classes['source-operate'], { disabled })}>
                {icUpdate}
                {icReset}
                {icUpload}
            </div>
        );
    }

    if (status === ENUM_DOWNLOAD_STATUS.TOBEUPDATE) {
        return (
            <div className={cx(classes['source-operate'], { disabled })}>
                {icUpdate}
                {icUpload}
                {icDelete}
            </div>
        );
    }

    return null;
}

export default React.memo(RenderOperation);
