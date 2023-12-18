import React, { useCallback, useEffect, useRef, useState } from 'react';
import { Modal, Select, Checkbox } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import i18next from '@dreamview/dreamview-lang';
import Logo from '@dreamview/dreamview-core/src/assets/welcome_guide_logov2.png';
import showModal, { ModalComponentProps } from '../../../../util/modal';
import useStyle from './useStyle';

enum KEY {
    GENERAL = 'general',
    ABOUT = 'about',
}

interface ISettingModal {
    dreamviewVersion: string;
    dockerVersion: string;
}

function Tabs(props: { tabs: KEY[]; activeKey: string; setActiveKey: (key: KEY) => void }) {
    const { classes, cx } = useStyle();
    const { tabs, activeKey, setActiveKey } = props;
    // const [active, setActive] = useState(KEY.GENERAL);
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

const tabs = [KEY.GENERAL, KEY.ABOUT];

function General() {
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
        </div>
    );
}

function About(props: { dremviewVersion: string; dockerVersion: string }) {
    const { classes, cx } = useStyle();
    const { t } = useTranslation('personal');
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
        </div>
    );
}

function SettingModal(props: ModalComponentProps & ISettingModal) {
    const { destroy } = props;
    const [activeKey, setActiveKey] = useState(KEY.GENERAL);

    const { classes, cx } = useStyle();

    const { t } = useTranslation('personal');

    const onClose = useCallback(() => {
        destroy();
    }, []);

    return (
        <Modal
            cancelButtonProps={{ style: { display: 'none' } }}
            okText={t('close')}
            width={962}
            onCancel={onClose}
            onOk={onClose}
            title={t('globalSetting')}
            open
        >
            <div className={classes['setting-root']}>
                <Tabs setActiveKey={setActiveKey} activeKey={activeKey} tabs={tabs} />
                <div className={classes['setting-right']}>
                    {activeKey === KEY.GENERAL && <General />}
                    {activeKey === KEY.ABOUT && (
                        <About dremviewVersion={props.dreamviewVersion} dockerVersion={props.dockerVersion} />
                    )}
                </div>
            </div>
        </Modal>
    );
}

export default (props: ISettingModal) => {
    showModal<ISettingModal>(props, SettingModal);
};
