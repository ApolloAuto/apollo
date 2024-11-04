// @ts-ignore
import React from 'react';
import { Story, StoryContext } from '@storybook/react';
import { Provider as ThemeProvider } from '@dreamview/dreamview-theme';
import GlobalStyles from '@dreamview/dreamview-core/src/style/globalStyles';
import { DndProvider } from 'react-dnd';
import { HTML5Backend } from 'react-dnd-html5-backend';
import { PanelLayoutStoreProvider } from '@dreamview/dreamview-core/src/store/PanelLayoutStore';
import { PanelCatalogProvider } from '@dreamview/dreamview-core/src/store/PanelCatalogStore';
import CombineContext from '@dreamview/dreamview-core/src/store/combineContext';
import { EventHandlersProvider } from '@dreamview/dreamview-core/src/store/EventHandlersStore';
import { MenuStoreProvider } from '@dreamview/dreamview-core/src/store/MenuStore';
import { HmiStoreProvider, PickHmiStoreProvider } from '@dreamview/dreamview-core/src/store/HmiStore';
import { WebSocketManagerProvider } from '@dreamview/dreamview-core/src/store/WebSocketManagerStore';
import InitAppData from '@dreamview/dreamview-core/src/InitAppData';
import { UserInfoStoreProvider } from '@dreamview/dreamview-core/src/store/UserInfoStore';
import { PanelInfoStoreProvider } from '@dreamview/dreamview-core/src/store/PanelInfoStore';
import {AppInitProvider} from "@dreamview/dreamview-core/src/store/AppInitStore";

function withContextProviders(Child: Story, context: StoryContext) {
    const Providers = [
        <AppInitProvider key='AppInitProvider' />,
        <EventHandlersProvider key='EventHandlersProvider' />,
        <WebSocketManagerProvider key='WebSocketManagerProvider' />,
        <UserInfoStoreProvider key='UserInfoStoreProvider' />,
        // 组件字典
        <PanelCatalogProvider key='PanelCatalogProvider' />,
        // 面板布局状态管理
        <PanelLayoutStoreProvider key='PanelLayoutStoreProvider' />,
        <MenuStoreProvider key='MenuStoreProvider' />,
        <HmiStoreProvider key='HmiStoreProvider' />,
        <PickHmiStoreProvider key='PickHmiStoreProvider' />,
        <PanelInfoStoreProvider key='PanelInfoStoreProvider' />,
    ];

    return (
        <ThemeProvider>
            <DndProvider backend={HTML5Backend}>
                <GlobalStyles />
                <CombineContext providers={Providers}>
                    <InitAppData />
                        <Child />
                </CombineContext>
            </DndProvider>
        </ThemeProvider>
    );
}

export const decorators = [withContextProviders];
export const loaders = [];
export const parameters = [];
