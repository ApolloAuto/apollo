import React, { useState, useEffect } from 'react';
import { makeStyles } from '@dreamview/dreamview-theme';
import { useAppInitContext, IRegistryItem, IAppInitStatus } from '@dreamview/dreamview-core/src/store/AppInitStore';
import { IconPark, Spin, useImagePrak } from '@dreamview/dreamview-ui';
import { LoadingOutlined } from '@ant-design/icons';

const useStyle = makeStyles((theme) => ({
    'dv-app-loading': {
        position: 'fixed',
        display: 'flex',
        alignItems: 'center',
        justifyContent: 'center',
        zIndex: theme.tokens.zIndex.app,
        top: 0,
        left: 0,
        right: 0,
        bottom: 0,
        background: theme.components.pageLoading.bgColor,
        '& img': {
            width: '650px',
            display: 'block',
            margin: '-150px auto 0',
        },
        '& .dreamview-progress': {
            width: '624px',
        },
    },
    'progress-label': {
        width: '50%',
        color: theme.components.pageLoading.color,
        textAlign: 'right',
        marginRight: '20px',
    },
    'progress-item': {
        display: 'flex',
        alignItems: 'center',
        marginBottom: theme.tokens.margin.speace,
        color: theme.components.pageLoading.color,
        '& .dreamview-progress': {
            display: 'inline',
            width: 'auto',
        },
        '& .init-success': {
            color: theme.tokens.font.color.success,
        },
    },
    'progress-item-success': {
        color: theme.tokens.font.color.success,
    },
    'progress-item-fail': {
        color: theme.tokens.font.color.error,
    },
}));
function Icon(props: { status: IAppInitStatus }) {
    const { status } = props;

    if (status === IAppInitStatus.DONE) {
        return <IconPark name='IcSucceed' className='init-success' />;
    }
    if (status === IAppInitStatus.LOADING) {
        return <Spin indicator={<LoadingOutlined style={{ fontSize: 16 }} spin />} />;
    }
    return <IconPark name='IcErrorMessage' />;
}

function PageLoading() {
    const { classes, cx } = useStyle();
    const { progressChangeHandler } = useAppInitContext();

    const [progress, setProgress] = useState<IRegistryItem[]>([]);

    useEffect(() => {
        progressChangeHandler((val: Record<string, IRegistryItem>) => {
            setProgress(Object.values(val).sort((next, prev) => next.order - prev.order));
        });
    }, []);

    const logo = useImagePrak('welcome_guide_logov2');

    return (
        <div className={classes['dv-app-loading']}>
            <div>
                <img alt='dreamviewplus' src={logo} />
                <div>
                    {progress.map((item) => (
                        <div
                            className={cx(classes['progress-item'], {
                                [classes['progress-item-success']]: item.status === IAppInitStatus.DONE,
                                [classes['progress-item-loading']]: item.status === IAppInitStatus.LOADING,
                                [classes['progress-item-fail']]: item.status === IAppInitStatus.FAIL,
                            })}
                            key={item.moduleName}
                        >
                            <div className={classes['progress-label']}>{item.moduleName}</div>
                            <Icon status={item.status} />
                            &nbsp;
                            {item.status === IAppInitStatus.LOADING && (
                                <>
                                    {item.progress}
                                    <>%</>
                                </>
                            )}
                        </div>
                    ))}
                </div>
            </div>
        </div>
    );
}

export default React.memo(PageLoading);
