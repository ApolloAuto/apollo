/* *****************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 **************************************************************************** */

import { createMakeAndWithStyles } from 'tss-react';
import React from 'react';
import { CSSObject } from '@emotion/react';
import { dreamviewColors } from './tokens/baseToken/colors';
import baseToken from './tokens/baseToken';
import components from './tokens/components';
import util from './tokens/util';

const defaultTheme = {
    colors: dreamviewColors.light,
    tokens: baseToken,
    components,
    util,
};

const context = React.createContext(null);

type IDefaultTheme = typeof defaultTheme;

interface ProviderProps {
    config?: Partial<IDefaultTheme>;
}

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

export const deepMerge = (origin: any, current: any) => {
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

export function Provider<T>(props: React.PropsWithChildren<ProviderProps>) {
    const { config } = props;
    const hoc = React.useMemo(
        () => ({
            ...createMakeAndWithStyles({
                useTheme: () => deepMerge(defaultTheme, config || {}) as Partial<IDefaultTheme> & T,
            }),
        }),
        [JSON.stringify(config)],
    );

    return <context.Provider value={hoc}>{props.children}</context.Provider>;
}

export function useThemeContext() {
    return React.useContext(context);
}

export const useMakeStyle = (styleFuc: (theme: IDefaultTheme, prop?: any) => CSSObject) => {
    const { makeStyles } = useThemeContext();
    const hoc = React.useMemo(() => makeStyles()(styleFuc), [makeStyles]);
    return hoc;
};
