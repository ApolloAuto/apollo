/* eslint-disable prettier/prettier */
import React, { useCallback, useEffect, useState } from 'react';
import { Modal, Select, useImagePrak, Checkbox, Radio, Button, message } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import { TFunction } from 'i18next';
import { useUserInfoStore } from '@dreamview/dreamview-core/src/store/UserInfoStore';
import i18next from '@dreamview/dreamview-lang';
import RcTable from 'rc-table';
import { noop } from 'lodash';
import showModal, { ModalComponentProps } from '../../../../util/modal';
import useStyle from './useStyle';
import Alert from './Alert';
import ImNotLogin from './not-login.png';

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

enum KEY {
    GENERAL = 'general',
    ABOUT = 'about',
    DEVICE = 'device',
    ACCOUNT = 'account',
}

function ButtomButton(props: { text: string; onClick: () => void; disabled?:boolean; loading?: boolean }) {
    const { classes } = useStyle();
    return (
        <Button disabled={props.disabled} className={classes['bottom-btn']} onClick={props.onClick || noop} loading={props.loading}>
            {props.text}
        </Button>
    );
}

interface ISettingModal {
    dreamviewVersion: string;
    dockerVersion: string;
    isLogin: boolean;
    userInfo: any;
    pluginApi: any;
    updateSubscribeAccount: () => void;
}

function Tabs(props: { tabs: KEY[]; activeKey: string; setActiveKey: (key: KEY) => void }) {
    const { classes, cx } = useStyle();
    const { tabs, activeKey, setActiveKey } = props;
    const { t } = useTranslation('personal');
    const onClick = (item: KEY) => {
        setActiveKey(item);
    };
    return (
        <div className={classes.tabs}>
            {tabs.map((item) => (
                <div
                    onClick={() => onClick(item)}
                    className={cx(classes['tabs-item'], { [classes.active]: activeKey === item })}
                    key={item}
                >
                    {t(item)}
                </div>
            ))}
        </div>
    );
}

const tabs = [KEY.GENERAL, KEY.ACCOUNT, KEY.ABOUT, KEY.DEVICE];

function General(props: { onClose: () => void }) {
    const { classes, cx } = useStyle();
    const { t } = useTranslation('personal');
    const options = [
        {
            label: '中文',
            value: 'zh',
        },
        {
            label: 'English',
            value: 'en',
        },
    ];
    const onChange = (val: string) => {
        if (val === 'zh') {
            i18next.changeLanguage('zh');
        } else {
            i18next.changeLanguage('en');
        }
    };
    return (
        <div>
            <div className={classes.laguage}>
                <p>{t('language')}</p>
                <Select onChange={onChange} options={options} placeholder='请选择' defaultValue={i18next.language} />
            </div>
            <ButtomButton onClick={props.onClose} text={t('close')} />
        </div>
    );
}

function About(props: { dremviewVersion: string; dockerVersion: string; onClose: () => void }) {
    const { classes, cx } = useStyle();
    const { t } = useTranslation('personal');
    const Logo = useImagePrak('welcome_guide_logov2');
    return (
        <div className={classes.about}>
            <img className={classes.logo} alt='apollo logo' src={Logo} />
            <div className={classes.aboutitem}>
                {t('dreamviewVersion')}
                &nbsp;
                {props.dremviewVersion}
            </div>
            <div className={classes.aboutitem}>
                {t('dockerVersion')}
                &nbsp;
                {props.dockerVersion}
            </div>
            <div className={classes.divider} />
            {/* <div className={cx(classes.blod, classes.aboutitem)}>{t('copyright')}</div>
            <div className={classes.aboutitem}>
                {t('copyrightDetail')}
                <a target='_blank' href='https://gitee.com/ApolloAuto/apollo/blob/master/LICENSE' rel='noreferrer'>
                    Apollo-2.0 License
                </a>
            </div> */}
            <ButtomButton onClick={props.onClose} text={t('close')} />
        </div>
    );
}

function Account(props: any) {
    const { pluginApi, updateSubscribeAccount, account } = props;
    const { classes } = useStyle();
    const { t } = useTranslation('personal');

    const onChange = (v: any) => {
        pluginApi?.changeSubscribe(v.target.value).then((r: any) => {
            updateSubscribeAccount();
        }).catch((err: any) => {
            messageError(err);
        });
    };
    const [accountList, setAccountList] = useState<any>([]);

    useEffect(() => {
        pluginApi?.getSubscribeList().then((r: any) => {
            setAccountList(r);
        }).catch((err: any) => {
            messageError(err);
        });
    }, [pluginApi]);

    return (
        <div>
            <div className={classes['account-flex']}>
                <div style={{ whiteSpace: 'nowrap' }}>{t('institutionalAccounts')}</div>
                <div>
                    {accountList?.length
                        ? accountList.map((item: any) => (
                            <Radio
                                onChange={onChange}
                                checked={account?.subscriber?.subscriberId === item.subscriber.subscriberId}
                                key={item.subscriber.subscriberId}
                                value={item.subscriber.subscriberId}
                            >
                                {item.subscriber.subName}
                            </Radio>
                        ))
                        : '--'}
                </div>
            </div>
            <div className={classes['account-flex']}>
                {t('personal')}
                ：
                <div>
                    <Radio onChange={onChange} checked={!account?.subscriber?.subscriberId} value='personal'>
                        {t('personal')}
                    </Radio>
                </div>
            </div>
            <ButtomButton onClick={props.onClose} text='close' />
        </div>
    );
}

const columns = (
    t: TFunction<'personal'>,
    classes: any,
    activeDevice: string,
    onSelectDevice: (device: string) => void,
) => [
    {
        title: t('tableIndex'),
        key: 'deviceId',
        width: 101,
        render(value: any, item: any, index: any) {
            return (
                <>
                    <Checkbox
                        disabled={!item.productsDeviceInfo?.some((sub: any) => sub.isAuthenticated === 1)}
                        onChange={() => onSelectDevice(item.deviceId)}
                        checked={activeDevice === item.deviceId}
                    />
                    &nbsp;&nbsp;
                    {index + 1}
                </>
            );
        },
    },
    {
        title: t('tableDeviceType'),
        width: 108,
        dataIndex: 'deviceType',
        key: 'deviceType',
    },
    {
        title: t('tableVehicleType'),
        width: 122,
        dataIndex: 'deviceModel',
        key: 'deviceModel',
    },
    {
        title: t('tableVehicleOrder'),
        width: 215,
        dataIndex: 'logicalDeviceInfo',
        key: 'deviceId',
        render(val: any) {
            return <span>{val?.carNumber}</span>;
        },
    },
    {
        title: t('tableProduct'),
        dataIndex: 'productsDeviceInfo',
        width: 187,
        key: 'productsDeviceInfo',
        render(value: any) {
            const hasBind = !!value?.length;

            if (!hasBind) {
                return (
                    <div className={classes['device-product']}>
                        <div className={classes['device-tag']}>未绑定</div>
                    </div>
                );
            }
            return (
                <div className={classes['device-product']}>
                    {value.map((item: any) => (
                        <div className={classes['device-tag']} key={item}>
                            {item.productNameCn}
                        </div>
                    ))}
                </div>
            );
        },
    },
];

function Device(props: { account: any; isLogin: boolean; userInfo: any; onClose: () => void; pluginApi: any }) {
    const { pluginApi } = props;
    const { t } = useTranslation('personal');
    const { classes, cx } = useStyle();
    const [activeDevice, setActiveDevice] = useState<string>();
    const [deviceList, setDeviceList] = useState<any>([]);
    const [hasBindLogicDevice, setHasBindLogicDevice] = useState(false);
    const [deviceInfo, setDeviceInfo] = useState<any>();
    const [registerLoading, setRegisterLoading] = useState(false);
    const onSelectDevice = (device: string) => {
        setActiveDevice(device === activeDevice ? null : device);
    };

    function updateAuthorizationInfo() {
        return pluginApi?.getAuthorizationInfo().then((r: any) => {
            setDeviceInfo(r);
            const hasBind = r?.logicalDevice?.carNumber;
            setHasBindLogicDevice(hasBind);
            return r;
        }).catch((err: any) => {
            messageError(err);
            return Promise.reject();
        });
    }

    useEffect(() => {
        updateAuthorizationInfo()
            .then((r: any) => {
                const hasBind = r?.logicalDevice?.carNumber;
                setHasBindLogicDevice(hasBind);
                if (!hasBind) {
                    pluginApi?.getCloudDeviceList().then((subR: any) => {
                        setDeviceList((subR.contents || []).filter((item: any) => item?.deviceType !== 'V2X'));
                    }).catch((err: any) => {
                        messageError(err);
                    });
                }
            })
            .catch(() => {
                pluginApi?.getCloudDeviceList().then((subR: any) => {
                    setDeviceList((subR.contents || []).filter((item: any) => item?.deviceType !== 'V2X'));
                }).catch((err: any) => {
                    messageError(err);
                });
            });
    }, [pluginApi]);

    const onRegister = () => {
        setRegisterLoading(true);
        pluginApi
            ?.checkInHwDevice(activeDevice)
            .then(() => {
                setRegisterLoading(false);
                updateAuthorizationInfo();
            })
            .catch((err: any) => {
                messageError(err);
                setRegisterLoading(false);
            });
    };
    const buttonPropsMap: any = {
        // 未申请
        0: '',
        // 申请中
        1: '',
        // 驳回
        2: '',
        // 待激活
        3: {
            onClick: () => {
                pluginApi?.updateLiscence().then(() => {
                    updateAuthorizationInfo();
                }).catch((err: any) => {
                    messageError(err);
                });
            },
            text: t('activation'),
        },
        // 已激活
        4: '',
        // 激活失败
        5: {
            onClick: () => {
                pluginApi?.updateLiscence().then(() => {
                    updateAuthorizationInfo();
                }).catch((err: any) => {
                    messageError(err);
                });
            },
            text: t('reActivation'),
        },
        // 已失效
        6: {
            onClick: () => {
                pluginApi?.updateLiscence().then(() => {
                    updateAuthorizationInfo();
                }).catch((err: any) => {
                    messageError(err);
                });
            },
            text: t('updateCertificate'),
        },
        // 待更新
        7: {
            onClick: () => {
                pluginApi?.updateLiscence().then(() => {
                    updateAuthorizationInfo();
                }).catch((err: any) => {
                    messageError(err);
                });
            },
            text: t('updateCertificate'),
        },
    };
    const statusMap: any = {
        0: t('notApply'),
        1: t('inApplication'),
        2: t('reject'),
        3: t('waitForActive'),
        4: t('activated'),
        5: t('activationFailed'),
        6: t('invalid'),
        7: t('waitingForUpdates'),
    };
    const status = statusMap[deviceInfo?.physicalDevice?.licenseStatus as any];
    const buttonProps = buttonPropsMap[deviceInfo?.physicalDevice?.licenseStatus as any];

    if (!props.isLogin) {
        return (
            <div className={classes['not-login']}>
                <img src={ImNotLogin} alt='' />
                <p>{t('noLogin')}</p>
                <div>{t('notLoginTips')}</div>
                <ButtomButton onClick={props.onClose} text={t('close')} />
            </div>
        );
    }
    if (!props.account) {
        return (
            <div className={classes['not-login']}>
                <img src={ImNotLogin} alt='' />
                <p>{t('notAccount')}</p>
                <div>{t('notAccountTips')}</div>
                <ButtomButton onClick={props.onClose} text={t('close')} />
            </div>
        );
    }
    if (!hasBindLogicDevice) {
        return (
            <div>
                <Alert text={t('deviceTips')} />
                <RcTable
                    data={deviceList}
                    className={classes['device-table']}
                    scroll={{
                        y: 260,
                    }}
                    columns={columns(t, classes, activeDevice, onSelectDevice)}
                />
                <ButtomButton disabled={!activeDevice} loading={registerLoading} onClick={onRegister} text={t('deviceRegistration')} />
            </div>
        );
    }
    return (
        <div>
            <div className={classes['device-flex']}>
                <div className={classes['device-label']}>
                    {t('deviceType')}
                    ：
                </div>
                <div className={classes['device-value']}>{deviceInfo?.logicalDevice?.deviceType}</div>
            </div>
            <div className={classes['device-flex']}>
                <div className={classes['device-label']}>
                    {t('carType')}
                    ：
                </div>
                <div className={classes['device-value']}>{deviceInfo?.logicalDevice?.deviceModel}</div>
            </div>
            <div className={classes['device-flex']}>
                <div className={classes['device-label']}>
                    {t('vehicleOrder')}
                    ：
                </div>
                <div className={classes['device-value']}>{deviceInfo?.logicalDevice?.carNumber}</div>
            </div>
            <div className={classes['device-flex']}>
                <div className={classes['device-label']}>
                    {t('cpu')}
                    ：
                </div>
                <div className={classes['device-value']}>{deviceInfo?.physicalDevice?.cpuType}</div>
            </div>
            <div className={classes['device-flex']}>
                <div className={classes['device-label']}>
                    {t('productLine')}
                    ：
                </div>
                <div className={classes['device-value']}>
                    {deviceInfo?.productLine?.map((item: any) => (
                        <div key={item.productNameCn} className={cx(classes['device-tag'], classes['float-left'])}>
                            {item.productNameCn}
                        </div>
                    ))}
                </div>
            </div>
            <div className={classes['device-flex']}>
                <div className={classes['device-label']}>
                    {t('licenseStatus')}
                    ：
                </div>
                <div className={classes['device-value']}>{status}</div>
            </div>
            {buttonProps && <ButtomButton {...buttonProps} />}
        </div>
    );
}

function SettingModal(props: ModalComponentProps & ISettingModal) {
    const { destroy, pluginApi, updateSubscribeAccount: propUpdateSubscribeAccount } = props;
    const [activeKey, setActiveKey] = useState(KEY.GENERAL);
    const [account, setAccountInfo] = useState(null);

    const { classes } = useStyle();

    const { t } = useTranslation('personal');

    const onClose = useCallback(() => {
        destroy();
    }, []);

    const updateSubscribeAccount = () => {
        pluginApi
            ?.getSubscribeAccountInfo()
            .then((r: any) => {
                setAccountInfo(r);
            });
        propUpdateSubscribeAccount();
    };

    useEffect(() => {
        updateSubscribeAccount();
    }, [pluginApi]);

    return (
        <Modal
            footer={null}
            okText={t('close')}
            width={962}
            rootClassName={classes['setting-modal']}
            onCancel={onClose}
            onOk={onClose}
            title={t('globalSetting')}
            open
        >
            <div className={classes['setting-root']}>
                <Tabs setActiveKey={setActiveKey} activeKey={activeKey} tabs={tabs} />
                <div className={classes['setting-right']}>
                    {activeKey === KEY.GENERAL && <General onClose={destroy} />}
                    {activeKey === KEY.ACCOUNT && (
                        <Account
                            onClose={destroy}
                            account={account}
                            updateSubscribeAccount={updateSubscribeAccount}
                            pluginApi={props.pluginApi}
                        />
                    )}
                    {activeKey === KEY.ABOUT && (
                        <About
                            onClose={destroy}
                            dremviewVersion={props.dreamviewVersion}
                            dockerVersion={props.dockerVersion}
                        />
                    )}
                    {activeKey === KEY.DEVICE && (
                        <Device
                            account={account}
                            pluginApi={props.pluginApi}
                            onClose={destroy}
                            isLogin={props.isLogin}
                            userInfo={props.userInfo}
                        />
                    )}
                </div>
            </div>
        </Modal>
    );
}

export default (props: ISettingModal) => {
    showModal<ISettingModal>(props, SettingModal);
};
