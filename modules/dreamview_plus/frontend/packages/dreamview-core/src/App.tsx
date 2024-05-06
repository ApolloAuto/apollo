import React from 'react';
import { DndProvider } from 'react-dnd';
import { HTML5Backend } from 'react-dnd-html5-backend';
import { PanelLayoutStoreProvider } from './store/PanelLayoutStore';
import { ThemeProvider } from './store/ThemeProviderStore';
import CombineContext from './store/combineContext';
import { PanelCatalogProvider } from './store/PanelCatalogStore';
import PageLayout from './components/PageLayout';
import GlobalStyles from './style/globalStyles';
import { EventHandlersProvider } from './store/EventHandlersStore';
import { MenuStoreProvider } from './store/MenuStore';
import { HmiStoreProvider, PickHmiStoreProvider } from './store/HmiStore';
import { WebSocketManagerProvider } from './store/WebSocketManagerStore';
import InitAppData from './InitAppData';
import { UserInfoStoreProvider } from './store/UserInfoStore';
import 'mac-scrollbar/dist/mac-scrollbar.css';
import { PanelInfoStoreProvider } from './store/PanelInfoStore';
import { AppInitProvider } from './store/AppInitStore';

export function App() {
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
                    <PageLayout />
                </CombineContext>
            </DndProvider>
        </ThemeProvider>
    );
}
