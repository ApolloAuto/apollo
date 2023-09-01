import React, { useContext, useMemo, useState } from 'react';
import { Meta, StoryObj } from '@storybook/react';
import { createMakeAndWithStyles } from 'tss-react';
import { CSSObject } from '@emotion/react';
import { Select } from 'antd';

type IDefaultTheme = {
    color1: string;
    color2: string;
};

interface ProviderProps {
    config?: Partial<IDefaultTheme>;
}

const defaultThemeColor = {
    color1: '#F3FDB0',
    color2: '#FEA443',
};

const firstThemeColor = {
    color1: '#705E78',
    color2: '#A5AAA3',
};

const secondThemeColor = {
    color1: '#812F33',
    color2: '#90CBFB',
};

const filterAdd = (origin: any, current: any) => {
    const originKeys = Object.keys(origin);
    const currentKeys = Object.keys(current);
    const addKeys = currentKeys.filter((key: string) => !originKeys.includes(key));
    return addKeys.reduce(
        (result: any, currentKey: any) => ({
            ...result,
            [currentKey]: current[currentKey],
        }),
        {},
    );
};

const deepMerge = (origin: any, current: any) => {
    const originKeys = Object.keys(origin);

    const nextValue = (key: any, originValue: any, currentValue: any): any => {
        if (!(key in currentValue)) {
            return originValue[key];
        }

        const v = originValue[key];

        if (!v) {
            return null;
        }

        if (typeof currentValue[key] === 'string') {
            return currentValue[key];
        }
        return deepMerge(originValue[key], currentValue[key]);
    };

    const mergeResult = originKeys.reduce(
        (result, item) => ({
            ...result,
            [item]: nextValue(item, origin, current),
        }),
        {},
    );
    const addResult = filterAdd(origin, current);
    return {
        ...mergeResult,
        ...addResult,
    };
};

const themeContext = React.createContext(null);

function useThemeContext() {
    return React.useContext(themeContext);
}

function ThemeProvider<T>(props: React.PropsWithChildren<ProviderProps>) {
    const { config } = props;

    const hoc = React.useMemo(
        () => ({
            ...createMakeAndWithStyles({
                useTheme: () => deepMerge({}, config || {}) as Partial<IDefaultTheme> & T,
            }),
        }),
        [config],
    );

    return <themeContext.Provider value={hoc}>{props.children}</themeContext.Provider>;
}

const useMakeStyle = (styleFuc: (theme: IDefaultTheme, prop?: any) => CSSObject) => {
    const { makeStyles } = useThemeContext();
    const hoc = React.useMemo(() => makeStyles()(styleFuc), [makeStyles]);
    return hoc;
};

interface ThemeContentProps {
    changeTheme: (themeVal: IDefaultTheme) => void;
}

function ThemeContent(props: ThemeContentProps) {
    const { changeTheme } = props;

    const options = [
        {
            value: 'defaultThemeColor',
            label: '默认主题颜色',
        },
        {
            value: 'firstThemeColor',
            label: '主题颜色1',
        },
        {
            value: 'secondThemeColor',
            label: '主题颜色2',
        },
    ];

    const onChange = (val: any) => {
        switch (val) {
            case 'defaultThemeColor':
                changeTheme(defaultThemeColor);
                break;
            case 'firstThemeColor':
                changeTheme(firstThemeColor);
                break;
            case 'secondThemeColor':
                changeTheme(secondThemeColor);
                break;
            default:
                changeTheme(defaultThemeColor);
        }
    };

    const { classes } = useMakeStyle((themeVal) => ({
        color1: {
            color: themeVal.color1,
        },
        color2: {
            color: themeVal.color2,
        },
    }))();

    return (
        <div>
            <Select defaultValue='defaultThemeColor' options={options} onChange={onChange} style={{ width: 200 }} />
            <div className={classes.color1}>主题文本1</div>
            <div className={classes.color2}>主题文本2</div>
        </div>
    );
}

export function CommonTheme() {
    const [themeState, setThemeState] = useState(defaultThemeColor);

    const changeTheme = (themeVal: IDefaultTheme) => {
        setThemeState(themeVal);
    };
    return (
        <ThemeProvider config={themeState}>
            <ThemeContent changeTheme={changeTheme} />
        </ThemeProvider>
    );
}

const meta: Meta<typeof CommonTheme> = {
    title: 'Theme/CommonTheme',
    component: CommonTheme,
};

export default meta;
