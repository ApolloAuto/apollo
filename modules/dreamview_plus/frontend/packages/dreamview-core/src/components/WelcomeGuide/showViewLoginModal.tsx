/* eslint-disable prettier/prettier */
import React from 'react';
import { Modal, useImagePrak } from '@dreamview/dreamview-ui';
import showModal, { WithModalComponentProps } from '@dreamview/dreamview-core/src/util/modal';
import { useTranslation } from 'react-i18next';
import { TFunction } from 'i18next';
import useStyle from './useStyle';

function welcomeGuideLoginStepsArr(translation: TFunction, imgSrc: string[]) {
    const [
        viewLoginStepFirst, viewLoginStepSecond, viewLoginStepThirdOne, viewLoginStepThirdTwo, viewLoginStepFourth,
    ] = imgSrc;
    return [
        {
            loginStepsItemText: translation('viewLoginStepOne'),
            loginStepsItemImage: viewLoginStepFirst,
        },
        {
            loginStepsItemText: translation('viewLoginStepTwo'),
            loginStepsItemImage: viewLoginStepSecond,
        },
        {
            loginStepsItemText: translation('viewLoginStepThree'),
            loginStepsItemImage: viewLoginStepThirdOne,
        },
        {
            loginStepsItemText: '',
            loginStepsItemImage: viewLoginStepThirdTwo,
        },
        {
            loginStepsItemText: translation('viewLoginStepFour'),
            loginStepsItemImage: viewLoginStepFourth,
        },
    ];
}

type LoginStepsItemProps = {
    loginStepsItemText: string;
    loginStepsItemImage: string;
};

function LoginStepsItem(props: LoginStepsItemProps) {
    const { classes } = useStyle();
    const { loginStepsItemText, loginStepsItemImage } = props;

    return (
        <div key={loginStepsItemText} className={classes['welcome-guide-login-container']}>
            {loginStepsItemText ? (
                <div className={classes['welcome-guide-login-content-text']}>{loginStepsItemText}</div>
            ) : (
                <div className={classes['welcome-guide-login-content-text']} />
            )}
            <div
                className={classes['welcome-guide-login-content-image']}
                style={{ backgroundImage: `url(${loginStepsItemImage})` }}
            />
        </div>
    );
}

function WelcomeGuide(props: WithModalComponentProps) {
    const { destroy } = props;
    const { classes } = useStyle();
    const { t } = useTranslation('guide');
    const imgSrc = useImagePrak([
        'view_login_step_first',
        'view_login_step_second',
        'view_login_step_third_one',
        'view_login_step_third_two',
        'view_login_step_fourth',
    ]);
    return (
        <Modal
            width={1000}
            title={t('loginTip')}
            footer={null}
            open
            onCancel={destroy}
            onOk={destroy}
            className={classes['welcome-guide-login-modal']}
        >
            {welcomeGuideLoginStepsArr(t, imgSrc).map((item, index) => (
                <LoginStepsItem
                    loginStepsItemText={item.loginStepsItemText}
                    loginStepsItemImage={item.loginStepsItemImage}
                    key={item.loginStepsItemText}
                />
            ))}
        </Modal>
    );
}

export default () => {
    showModal({}, WelcomeGuide);
};
