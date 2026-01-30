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
    return React.useContext(context);
}

function useTheme() {
    return React.useContext(context).tokens;
}

const { makeStyles: initUseStyle } = createMakeAndWithStyles<IDefaultTheme>({
    useTheme,
});

export const makeStylesWithProps = initUseStyle;

export const makeStyles = initUseStyle();
