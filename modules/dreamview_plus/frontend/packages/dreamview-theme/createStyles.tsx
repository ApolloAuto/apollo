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
import React, { useRef } from 'react';
import isEqual from 'lodash/isEqual';
import { CSSObject } from '@emotion/react';
import { light, IDefaultTheme, ITheme, getToken } from './tokens/baseToken';

const context = React.createContext(null);

interface ProviderProps {
    theme?: ITheme;
}

const defaultTheme = {
    theme: 'light',
    tokens: getToken('light'),
    ...createMakeAndWithStyles({
        useTheme: () => light,
    }),
};

export function Provider(props: React.PropsWithChildren<ProviderProps>) {
    const { theme = 'light' } = props;
    const prevHoc = useRef(defaultTheme);
    const prevTheme = useRef('');

    const hoc = React.useMemo(() => {
        if (!isEqual(prevTheme.current, theme)) {
            prevHoc.current = {
                ...createMakeAndWithStyles({
                    useTheme: () => getToken(theme),
                }),
                theme,
                tokens: getToken(theme),
            };
        }
        return prevHoc.current;
    }, [theme]);

    prevTheme.current = theme;

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
