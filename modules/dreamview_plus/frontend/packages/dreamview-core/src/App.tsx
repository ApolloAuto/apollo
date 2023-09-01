import React from 'react';
import { Provider as ThemeProvider } from '@dreamview/dreamview-theme';
import { DndProvider } from 'react-dnd';
import { HTML5Backend } from 'react-dnd-html5-backend';
import { GlobalStyles, CSSInterpolation } from 'tss-react';
import { PanelLayoutStoreProvider } from './store/PanelLayoutStore';
import CombineContext from './store/combineContext';
import { PanelCatalogProvider } from './store/PanelCatalogStore';
import PageLayout from './components/PageLayout';
import globalStyles from './style/globalStyles';
import { EventHandlersProvider } from './store/EventHandlersStore';
import { MenuStoreProvider } from './store/MenuStore';
import { HmiStoreProvider, PickHmiStoreProvider } from './store/HmiStore';
import { WebSocketManagerProvider } from './store/WebSocketManagerStore';
import InitAppData from './InitAppData';
import { UserInfoStoreProvider } from './store/UserInfoStore';
import 'mac-scrollbar/dist/mac-scrollbar.css';
import { PanelInfoStoreProvider } from './store/PanelInfoStore';

export function App() {
    const Providers = [
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
            <GlobalStyles styles={globalStyles as CSSInterpolation} />
            <DndProvider backend={HTML5Backend}>
                <CombineContext providers={Providers}>
                    <InitAppData />
                    <PageLayout />
                </CombineContext>
            </DndProvider>
        </ThemeProvider>
    );
}
