import React from 'react';
import { Meta, StoryObj } from '@storybook/react';
import i18next from 'i18next';
import LanguageDetector from 'i18next-browser-languagedetector';
import { initReactI18next, useTranslation } from 'react-i18next';
import { Select } from 'antd';
import * as en from './language-i18next/en/language';
import * as zh from './language-i18next/zh/language';

const translations = { en, zh };

async function initI18n(resources = translations): Promise<void> {
    await i18next
        .use(LanguageDetector)
        .use(initReactI18next)
        .init({
            resources,
            lng: 'en',
            interpolation: {
                escapeValue: false,
            },
        });
}

initI18n();

export function CommonLanguage() {
    const { t: tCommon } = useTranslation('language');

    const options = [
        {
            value: 'en',
            label: 'English',
        },
        {
            value: 'zh',
            label: '中文',
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
            <Select defaultValue='en' options={options} onChange={onChange} style={{ width: 100 }} />
            <div style={{ width: 100, textAlign: 'center' }}>
                <div>{tCommon('title')}</div>
                <div>{tCommon('paragraph1')}</div>
                <div>{tCommon('paragraph2')}</div>
                <div>{tCommon('paragraph3')}</div>
                <div>{tCommon('footer')}</div>
            </div>
        </div>
    );
}

const meta: Meta<typeof CommonLanguage> = {
    title: 'Language/CommonLanguage',
    component: CommonLanguage,
};

export default meta;
