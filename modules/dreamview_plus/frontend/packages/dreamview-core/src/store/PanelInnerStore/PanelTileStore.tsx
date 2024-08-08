import React, { useMemo, useState, useContext } from 'react';
import EventEmitter from 'eventemitter3';
import { MosaicContext, MosaicPath } from 'react-mosaic-component';
import { FullScreenFnRef } from './type';

const context = React.createContext(null);

export function usePanelTileContext() {
    return React.useContext(context);
}

interface PanelTileProviderProps {
    path: MosaicPath;
    enterFullScreen: () => void;
    exitFullScreen: () => void;
    fullScreenFnObj: FullScreenFnRef;
}

export function PanelTileProvider(props: React.PropsWithChildren<PanelTileProviderProps>) {
    const { path, enterFullScreen, exitFullScreen, fullScreenFnObj } = props;
    const { mosaicActions } = useContext(MosaicContext);
    const [beforeCloseFunc, setBeforeCloseFunc] = useState(null);

    const contextValue = useMemo(
        () => ({
            eventEmitter: new EventEmitter(),
            onBeforeClosePanel: (callback: () => Promise<any>) => {
                setBeforeCloseFunc(() => callback);
            },
            onClosePanel: () => {
                if (beforeCloseFunc) {
                    beforeCloseFunc()
                        .then(() => {
                            mosaicActions.remove(path);
                        })
                        .catch(() => {
                            console.log('panel closed cancelled');
                        })
                        .finally(() => {
                            mosaicActions.remove(path);
                        });
                } else {
                    mosaicActions.remove(path);
                }
            },
            enterFullScreen,
            exitFullScreen,
            fullScreenFnObj,
        }),
        [mosaicActions, path, beforeCloseFunc, enterFullScreen, exitFullScreen],
    );

    return <context.Provider value={contextValue}>{props.children}</context.Provider>;
}
