import React, { PropsWithChildren, useMemo, useState } from 'react';
import { Provider } from '@dreamview/dreamview-theme';
import { useLocalStorageState } from '../util/storageManager';

const context = React.createContext(null);

export function ThemeProvider(props: PropsWithChildren) {
    const [theme, setTheme] = useLocalStorageState('dreamview-ui-theme', 'drak');
    const contextValue = useMemo(
        () => ({
            theme,
            setTheme,
        }),
        [],
    );

    return (
        <Provider theme={theme}>
            <context.Provider value={contextValue}>{props.children}</context.Provider>
        </Provider>
    );
}

export function useChangeTheme() {
    return React.useContext(context).setTheme;
}
