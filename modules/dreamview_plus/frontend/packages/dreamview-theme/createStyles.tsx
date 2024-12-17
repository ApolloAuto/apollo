/* eslint-disable react-hooks/rules-of-hooks */
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
import React, { useMemo } from 'react';
import { cloneDeep } from 'lodash';
import { light, IDefaultTheme, drak, ITheme } from './tokens/baseToken';

const context = React.createContext<{
    theme: ITheme;
    tokens: IDefaultTheme;
}>({
    theme: 'light',
    tokens: light,
});

interface ProviderProps {
    theme?: ITheme;
}

export function Provider(props: React.PropsWithChildren<ProviderProps>) {
    const { theme = 'light' } = props;

    const values = useMemo(
        () => ({
            theme,
            tokens: {
                light,
                drak,
            }[theme],
        }),
        [theme],
    );

    return <context.Provider value={values}>{props.children}</context.Provider>;
}

export function useThemeContext() {
    let ctx;
    try {
        ctx = React.useContext(context);
    } catch (err) {
        console.log('err', err);
    }
    if (!ctx) {
        console.warn('@dreamview/dreamview-theme context missing');
    }
    return ctx;
}

interface ExpandProviderProps<T> {
    theme?: ITheme;
    expand?: {
        light: T;
        drak: T;
    };
}

export function ExpandProvider<T>(props: ExpandProviderProps<T> & { children?: any }) {
    const { expand } = props;
    const themeContext = useThemeContext();

    const theme = props.theme || themeContext?.theme;
    if (!themeContext) {
        console.warn('@dreamview/dreamview-theme context missing');
    }

    const values = useMemo(() => {
        const lightClone = cloneDeep(light);
        const drakClone = cloneDeep(drak);
        if (expand?.light) {
            Object.assign(lightClone.components, expand.light);
        }
        if (expand?.drak) {
            Object.assign(drakClone.components, expand.drak);
        }
        return {
            theme,
            tokens: {
                light: lightClone,
                drak: drakClone,
            }[theme],
        };
    }, [theme]);

    return <context.Provider value={values}>{props.children}</context.Provider>;
}

function useTheme() {
    return React.useContext(context)?.tokens;
}

const { makeStyles: initUseStyle } = createMakeAndWithStyles<IDefaultTheme>({
    useTheme,
});

export const makeStylesWithProps = initUseStyle;

export const makeStyles = initUseStyle();
