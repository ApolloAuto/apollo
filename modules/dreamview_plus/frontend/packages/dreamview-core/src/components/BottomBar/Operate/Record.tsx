import React, { useCallback, useEffect, useState, useRef } from 'react';
import { Input, Modal } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { makeStyles, useThemeContext } from '@dreamview/dreamview-theme';
import showModal, { WithModalComponentProps } from '@dreamview/dreamview-core/src/util/modal';
import {
    usePickHmiStore,
    ENUM_DATARECORD_PROCESS_STATUS,
    ENUM_RTKRECORD_PROCESS_STATUS,
    HMIModeOperation,
} from '@dreamview/dreamview-core/src/store/HmiStore';
import useWebSocketServices from '@dreamview/dreamview-core/src/services/hooks/useWebSocketServices';
import dayjs from 'dayjs';
import useStyle from './useStyle';

function usePending(callback: () => Promise<any>): [boolean, () => void] {
    const [pendding, setPendding] = useState(false);

    const onDo = useCallback(() => {
        if (pendding) {
            return;
        }
        setPendding(true);
        callback()
            .then(() => {
                setPendding(false);
            })
            .catch(() => {
                setPendding(false);
            });
    }, [pendding, callback]);

    return [pendding, onDo];
}

const useModalStyle = makeStyles((theme) => ({
    'record-modal': {
        '& .dreamview-modal-footer': {
            textAlign: 'center',
        },
        '& .dreamview-modal-footer button': {
            padding: '9px 18px',
            height: 'auto',
            borderRadius: '8px',
            '&:first-of-type': {
                marginRight: theme.tokens.margin.speace3,
            },
        },
    },
    'record-modal-item': {
        display: 'flex',
        alignItems: 'center',
        position: 'relative',
    },
    'record-modal-label': {
        color: theme.tokens.colors.fontColor5,
        whiteSpace: 'nowrap',
        ...theme.tokens.typography.content,
        paddingRight: theme.tokens.margin.speace,
        width: 100,
        '&::after': {
            content: '":"',
        },
    },
    'record-error': {
        color: theme.tokens.colors.error2,
        fontSize: theme.tokens.font.size.sm,
        paddingLeft: '100px',
        marginTop: '4px',
        marginBottom: '-16px',
        minHeight: '12px',
        lineHeight: '12px',
    },
}));

interface ConfirmModalProps {
    onConfirmSave: (name: string) => Promise<any>;
    onCancelSave: () => void;
    defaultName: string;
    currentOperation: HMIModeOperation;
}
function ConfirmModal(props: WithModalComponentProps<ConfirmModalProps>) {
    const { destroy, defaultName, onConfirmSave, onCancelSave, currentOperation } = props;
    const { t } = useTranslation('bottomBar');
    const [name, setName] = useState(() => defaultName.replace(/\s/g, ''));
    const isLoad = useRef(true);
    const [error, setError] = useState('');

    const validate = () => {
        if (!name) {
            setError(t('nameEmpty'));
            return false;
        }

        if (/\s/.test(name)) {
            setError(t('nameHasWhitespace'));
            return false;
        }

        if (/[^0-9A-z_]/.test(name)) {
            setError(t('nameHasChinese'));
            return false;
        }

        setError('');
        return true;
    };

    const onOk = () => {
        const isValidate = validate();
        if (isValidate) {
            onConfirmSave(name)
                .then(() => {
                    destroy();
                })
                .catch((err) => {
                    setError(err?.data?.info?.message);
                });
        }
    };

    const onClose = () => {
        onCancelSave();
        destroy();
    };

    const { classes } = useModalStyle();

    const onChange = (e: any) => {
        setName(e?.target?.value || '');
    };

    useEffect(() => {
        if (isLoad.current) {
            isLoad.current = false;
        } else {
            validate();
        }
    }, [name]);

    return (
        <Modal
            okText={t('save')}
            cancelText={t('close')}
            cancelButtonProps={{
                ghost: true,
            }}
            width={400}
            rootClassName={classes['record-modal']}
            onOk={onOk}
            onCancel={onClose}
            open
            title={t('modalTitle')}
        >
            <div className={classes['record-modal-item']}>
                <span className={classes['record-modal-label']}>{t('labelName')}</span>
                <Input allowClear onChange={onChange} value={name} />
            </div>
            <div className={classes['record-error']}>{error}</div>
        </Modal>
    );
}

const showConfirmModal = (props: ConfirmModalProps) => showModal(props, ConfirmModal);

function RecordBtn() {
    const [hmi] = usePickHmiStore();
    const { mainApi, isMainConnected } = useWebSocketServices();
    const { classes, cx } = useStyle();
    const { t } = useTranslation('bottomBar');
    const { theme } = useThemeContext();
    const isDrak = theme === 'drak';
    const isInRecord = (() => {
        if (hmi.currentOperation === HMIModeOperation.WAYPOINT_FOLLOW) {
            return hmi.globalComponents?.RTKRecorder?.processStatus?.status === ENUM_RTKRECORD_PROCESS_STATUS.OK;
        }
        return hmi.globalComponents?.DataRecorder?.processStatus?.status === ENUM_DATARECORD_PROCESS_STATUS.OK;
    })();

    const [, onStartRecord] = usePending(
        useCallback(() => {
            if (isMainConnected) {
                return mainApi.startRecordPackets();
            }
            return Promise.reject();
        }, [isMainConnected]),
    );

    const onConfirmSave = useCallback(
        (name: string) => {
            if (isMainConnected) {
                return mainApi.saveRecordPackets(name);
            }
            return Promise.reject(new Error('websocket connect has closed'));
        },
        [isMainConnected],
    );

    const onCancelSave = useCallback(() => {
        if (isMainConnected) {
            mainApi.deleteRecordPackets();
        }
    }, [isMainConnected]);

    const [, onEndRecord] = usePending(
        useCallback(() => {
            if (isMainConnected) {
                return mainApi.stopRecordPackets().then(() => {
                    showConfirmModal({
                        onConfirmSave,
                        onCancelSave,
                        defaultName: [dayjs().format('YYYYMMDDHHmm'), hmi.currentVehicle, hmi.currentMap]
                            .filter(Boolean)
                            .join('_'),
                        currentOperation: hmi.currentOperation,
                    });
                });
            }
            return Promise.reject();
        }, [isMainConnected, hmi.currentVehicle, hmi.currentMap, hmi.currentOperation]),
    );

    // 重置record
    const onRecord = () => {
        if (isInRecord) {
            onEndRecord();
        } else {
            onStartRecord();
        }
    };

    return (
        <div onClick={onRecord} className={cx(classes['player-record-btn'])}>
            {isDrak ? (
                <svg
                    width='16px'
                    height='16px'
                    viewBox='0 0 16 16'
                    version='1.1'
                    xmlns='http://www.w3.org/2000/svg'
                    xmlnsXlink='http://www.w3.org/1999/xlink'
                >
                    <g strokeWidth='1' fill='none' fillRule='evenodd'>
                        <rect fill='currentColor' opacity='0' x='0' y='0' width='16' height='16' />
                        <circle stroke='currentColor' cx='8' cy='8' r='7' />
                        <circle fill={isInRecord ? '#F75660' : 'currentColor'} cx='8' cy='8' r='5' />
                    </g>
                </svg>
            ) : (
                <svg
                    width='16px'
                    height='16px'
                    viewBox='0 0 16 16'
                    version='1.1'
                    xmlns='http://www.w3.org/2000/svg'
                    xmlnsXlink='http://www.w3.org/1999/xlink'
                >
                    <g id='icon/ic_recording' stroke='none' strokeWidth='1' fill='none' fillRule='evenodd'>
                        <rect fill='currentColor' opacity='0' x='0' y='0' width='16' height='16' />
                        <circle stroke='currentColor' cx='8' cy='8' r='7' />
                        <circle fill={isInRecord ? '#F53145' : 'currentColor'} fillRule='nonzero' cx='8' cy='8' r='3' />
                    </g>
                </svg>
            )}
            <span className={classes['player-record-text']}>{isInRecord ? t('stopRecord') : t('record')}</span>
        </div>
    );
}

export default React.memo(RecordBtn);
